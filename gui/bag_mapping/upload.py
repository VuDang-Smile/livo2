"""
Upload Module
Chứa logic upload map lên backend server
"""
import zipfile
import json
import os
import time
from pathlib import Path
from datetime import datetime
from tkinter import messagebox

# Try to import requests
REQUESTS_AVAILABLE = False
requests = None
try:
    import requests
    REQUESTS_AVAILABLE = True
except (ImportError, SystemError, OSError):
    REQUESTS_AVAILABLE = False
    requests = None


class UploadManager:
    """Quản lý upload map lên backend"""
    
    def __init__(self, config, translator, logger_callback, progress_callback, 
                 pcd_processor, design_comparison, qr_save_callback):
        """
        Initialize upload manager
        
        Args:
            config: BagMappingConfig instance
            translator: Translator instance
            logger_callback: Function để log messages
            progress_callback: Function để update progress (value: 0-100)
            pcd_processor: PCDProcessor instance
            design_comparison: DesignMapComparison instance
            qr_save_callback: Function để save QR codes to JSON (returns path or None)
        """
        self.config = config
        self.translator = translator
        self.log = logger_callback
        self.set_progress = progress_callback
        self.pcd_processor = pcd_processor
        self.design_comparison = design_comparison
        self.save_qr_codes = qr_save_callback
    
    def start_upload(self, map_name_var, vehicle_info_var, design_file_path, 
                     comparison_threshold, root, btn_upload_callback):
        """
        Bắt đầu upload pipeline
        
        Args:
            map_name_var: Tkinter StringVar chứa map name
            vehicle_info_var: Tkinter StringVar chứa vehicle info
            design_file_path: Path đến design file (string hoặc None)
            comparison_threshold: Threshold cho design comparison
            root: Tkinter root window
            btn_upload_callback: Function để enable/disable upload button
        """
        from .utils import get_local_mac_no_colon
        from .utils import check_pcd_files
        
        pcd_dir = self.config.pcd_dir
        
        if not check_pcd_files(pcd_dir):
            messagebox.showerror(
                self.translator.get('dialog.no_pcd_files_title', 'No PCD files found'),
                self.translator.get('dialog.no_pcd_files_message', 
                    'No PCD files found in directory:\n{path}\n\nPlease run mapping first to create PCD files.')
                    .replace('{path}', str(pcd_dir))
            )
            return

        map_name = (map_name_var.get() or "").strip()
        vehicle_id = get_local_mac_no_colon()
        if vehicle_id:
            vehicle_info_var.set(vehicle_id)

        if not map_name:
            messagebox.showerror(
                self.translator.get('dialog.error', 'Error'),
                self.translator.get('dialog.map_name_required', 'Please enter Map Name before uploading.')
            )
            return

        if not vehicle_id:
            messagebox.showerror(
                self.translator.get('dialog.error', 'Error'),
                self.translator.get('dialog.vehicle_id_required', 'Cannot detect local MAC for Vehicle ID. Please check network interfaces.')
            )
            return

        # Kiểm tra bắt buộc phải chọn file thiết kế để so sánh
        if not design_file_path:
            messagebox.showerror(
                self.translator.get('dialog.error', 'Error'),
                self.translator.get('message.design_file_required', 
                    'Please select a design map file (PCD or OBJ) before uploading.\n\nDesign map comparison is required to ensure map quality.')
            )
            return
        
        # Validate design file exists
        design_path = Path(design_file_path)
        if not design_path.exists():
            messagebox.showerror(
                self.translator.get('dialog.error', 'Error'),
                self.translator.get('message.design_file_not_exists', 
                    'Design map file does not exist:\n{path}\n\nPlease select a valid design map file.')
                    .replace('{path}', str(design_path))
            )
            return

        btn_upload_callback(state='disabled')
        self.log(self.translator.get('log.pipeline_start', '🚀 Starting map processing and upload pipeline...'))
        
        import threading
        threading.Thread(
            target=self._run_upload_pipeline_thread,
            args=(map_name, vehicle_id, design_file_path, comparison_threshold, root, btn_upload_callback),
            daemon=True
        ).start()
    
    def _run_upload_pipeline_thread(self, map_name: str, vehicle_id: str, 
                                     design_file_path: str, comparison_threshold: float,
                                     root, btn_upload_callback):
        """Thread thực hiện full pipeline và upload"""
        start_time = time.time()
        
        try:
            # 1. Merge PCD
            self.log(self.translator.get('log.step_merge_pcd', 'Step 1/6: Merging PCD files...'))
            if not self.pcd_processor.merge_pcd_files(silent=True):
                self.log(self.translator.get('log.pipeline_stopped_merge', '❌ Pipeline stopped at Merge PCD step.'))
                root.after(0, lambda: btn_upload_callback(state='normal'))
                return
            
            root.after(0, lambda: self.set_progress(20))
            
            # 2. HBA Optimize
            self.log(self.translator.get('log.step_hba', 'Step 2/8: Running HBA Optimize...'))
            if not self.pcd_processor.run_hba_core():
                self.log(self.translator.get('log.pipeline_stopped_hba', '❌ Pipeline stopped at HBA step.'))
                root.after(0, lambda: btn_upload_callback(state='normal'))
                return
            
            root.after(0, lambda: self.set_progress(35))
            
            # 2.5. Compute Tunnel Direction
            self.log(self.translator.get('log.step_compute_tunnel_dir', 'Step 2.5/8: Computing tunnel direction...'))
            if not self.pcd_processor.compute_tunnel_direction():
                self.log(self.translator.get('log.pipeline_stopped_compute_dir', '❌ Pipeline stopped at Compute Tunnel Direction step.'))
                root.after(0, lambda: btn_upload_callback(state='normal'))
                return
            
            root.after(0, lambda: self.set_progress(40))
            
            # 2.6. Rotate PCD to +X
            self.log(self.translator.get('log.step_rotate_pcd', 'Step 2.6/8: Rotating PCD to align with +X axis...'))
            if not self.pcd_processor.rotate_pcd_to_positive_x():
                self.log(self.translator.get('log.pipeline_stopped_rotate', '❌ Pipeline stopped at Rotate PCD step.'))
                root.after(0, lambda: btn_upload_callback(state='normal'))
                return
            
            root.after(0, lambda: self.set_progress(45))
            
            # 3. SC Tile
            self.log(self.translator.get('log.step_sc', 'Step 3/8: Running ScanContext Tiling...'))
            if not self.pcd_processor.run_sc_core():
                self.log(self.translator.get('log.pipeline_stopped_sc', '❌ Pipeline stopped at ScanContext step.'))
                root.after(0, lambda: btn_upload_callback(state='normal'))
                return
            
            root.after(0, lambda: self.set_progress(60))
            
            # 4. Generate Floorplan
            self.log(self.translator.get('log.step_floorplan', 'Step 4/8: Generating floorplan...'))
            if not self.pcd_processor.generate_floorplan():
                self.log(self.translator.get('log.pipeline_stopped_floorplan', '❌ Pipeline stopped at Generate Floorplan step.'))
                root.after(0, lambda: btn_upload_callback(state='normal'))
                return
            
            root.after(0, lambda: self.set_progress(70))
            
            # 4.5. So sánh với bản thiết kế
            if design_file_path:
                self.log(self.translator.get('log.step_comparing_design_map', 
                    'Step 5/8: Comparing generated map with design map...'))
                generated_pcd_path = self.config.merged_map_dir / self.config.merged_pcd_name

                success, similarity = self.design_comparison.run_design_map_comparison(
                    generated_pcd_path
                )
                if not success:
                    self.log(self.translator.get('log.design_map_comparison_failed', 
                        '❌ Design map comparison failed. Stopping pipeline before upload.'))
                    root.after(0, lambda: btn_upload_callback(state='normal'))
                    return

                root.after(0, lambda: self.set_progress(75))

                # Nếu % khớp thấp hơn ngưỡng, hỏi người dùng
                if similarity < comparison_threshold:
                    warn_msg = (
                        self.translator.get('log.low_similarity_warning', 
                            '⚠️ Low similarity with design map: {similarity}% (threshold: {threshold}%)\n\nPossible drift detected. Do you want to continue upload anyway?')
                            .replace('{similarity}', f'{similarity:.1f}')
                            .replace('{threshold}', f'{comparison_threshold:.1f}')
                    )
                    self.log(
                        f"⚠️ Low similarity with design map: {similarity:.1f}% "
                        f"(threshold {comparison_threshold:.1f}%). Asking user..."
                    )
                    # Note: messagebox.askyesno cần được gọi trong main thread
                    import tkinter.messagebox as mb
                    should_continue = mb.askyesno(
                        self.translator.get('dialog.low_similarity_warning_title', 'Low similarity warning'), 
                        warn_msg
                    )
                    if not should_continue:
                        self.log("⛔ User cancelled upload due to low similarity with design map.")
                        root.after(0, lambda: btn_upload_callback(state='normal'))
                        return
                    else:
                        self.log(self.translator.get('log.user_continue_upload_low_similarity', 
                            '➡ User chose to continue upload despite low similarity.'))
            
            # 5. Save QR codes to JSON
            self.log(self.translator.get('log.step_save_qr', 'Step 6/8: Saving QR codes...'))
            if self.save_qr_codes:
                qr_json_path = self.save_qr_codes()
                if qr_json_path:
                    self.log(self.translator.get('log.qr_codes_saved', '✅ QR codes saved to: {path}').replace('{path}', str(qr_json_path)))
                else:
                    self.log(self.translator.get('log.no_qr_codes_to_save', '⚠️ No QR codes to save'))
            else:
                self.log(self.translator.get('log.no_qr_codes_to_save', '⚠️ No QR codes to save'))
            
            root.after(0, lambda: self.set_progress(80))
            
            # 6. Zip files
            self.log(self.translator.get('log.step_zip', 'Step 7/8: Compressing files (Zip)...'))
            zip_path = self.zip_map_folders(map_name, vehicle_id)
            if not zip_path:
                self.log(self.translator.get('log.pipeline_stopped_zip', '❌ Pipeline stopped at Zip step.'))
                root.after(0, lambda: btn_upload_callback(state='normal'))
                return
            
            root.after(0, lambda: self.set_progress(90))
            
            # 7. Upload to backend
            self.log(self.translator.get('log.step_upload', 'Step 8/8: Uploading to backend...'))
            upload_success, upload_info = self.upload_map_to_backend(zip_path, map_name, vehicle_id)
            if upload_success:
                if upload_info:
                    upload_msg = self.translator.get('log.upload_backend_success', '✅ Backend upload successful (upload_id: {upload_id})').replace('{upload_id}', str(upload_info))
                else:
                    upload_msg = "✅ Backend upload successful"
                self.log(upload_msg)
                root.after(0, lambda: btn_upload_callback(state='normal'))
            else:
                fail_reason = upload_info or "Unknown error"
                self.log(self.translator.get('log.upload_backend_failed', '⚠️ Backend upload failed: {reason}').replace('{reason}', str(fail_reason)))
                root.after(0, lambda: btn_upload_callback(state='normal'))
            
            root.after(0, lambda: self.set_progress(100))
            
            duration = time.time() - start_time
            self.log(self.translator.get('log.pipeline_complete', '🎉 Pipeline completed in {duration:.1f} seconds!').replace('{duration:.1f}', f'{duration:.1f}'))
            
            # Show success message
            upload_status = (
                f"Upload backend: OK (upload_id: {upload_info})" if upload_success and upload_info
                else "Upload backend: OK" if upload_success
                else f"Upload backend: LỖI ({upload_info})"
            )
            root.after(0, lambda: messagebox.showinfo(
                self.translator.get('dialog.pipeline_complete_title', 'Success'),
                self.translator.get('dialog.pipeline_complete_message', 
                    'Pipeline completed successfully!\n\nZip file: {filename}\n{upload_status}')
                    .replace('{filename}', zip_path.name)
                    .replace('{upload_status}', upload_status)
            ))
            
        except Exception as e:
            self.log(self.translator.get('log.error_in_pipeline', '❌ Error in pipeline: {error}').replace('{error}', str(e)))
            import traceback
            self.log(f"   Details: {traceback.format_exc()}")
            root.after(0, lambda: btn_upload_callback(state='normal'))
    
    def zip_map_folders(self, map_name: str = None, vehicle_id: str = None):
        """Zip các folder map và tạo map_metadata.json"""
        log_root = self.config.log_path
        merged_map_dir = self.config.merged_map_dir
        
        # Các folder cần zip
        folders_to_zip = [
            "merged_map",
            "hba_map", 
            "fastloc_map",
            "floorplan_2d"
        ]
        
        # Kiểm tra xem có folder nào tồn tại không
        existing_folders = []
        for folder_name in folders_to_zip:
            folder_path = log_root / folder_name
            if folder_path.exists() and any(folder_path.iterdir()):
                existing_folders.append(folder_name)
        
        # Kiểm tra file QR_detect.json
        qr_detect_file = self.config.qr_detect_json_path
        has_qr_file = qr_detect_file.exists() and qr_detect_file.stat().st_size > 0
        
        if not existing_folders and not has_qr_file:
            self.log(self.translator.get('log.zip_no_folders', '❌ No folders found to zip'))
            return None
        
        # Tạo output directory
        base_output_dir = Path(__file__).parent.parent / "output"
        v_num = 1
        while (base_output_dir / f"version1.{v_num}").exists():
            v_num += 1
        
        output_root = base_output_dir / f"version1.{v_num}"
        output_root.mkdir(parents=True, exist_ok=True)
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        zip_filename = f"map_v1.{v_num}_{timestamp}.zip"
        zip_path = output_root / zip_filename
        
        self.log("=" * 60)
        self.log(self.translator.get('log.zip_creating', '📦 Creating zip file: {filename}').replace('{filename}', zip_path.name))
        zip_items = existing_folders.copy()
        if has_qr_file:
            zip_items.append("QR_detect.json")
        self.log(self.translator.get('log.zip_folders', '📁 Items to zip: {folders}').replace('{folders}', ', '.join(zip_items)))
        
        try:
            # Đọc rotation metadata và tunnel direction nếu có
            rotation_metadata = None
            tunnel_direction_data = None
            
            rotation_meta_path = merged_map_dir / "merged_all_rotated_rotation_metadata.json"
            if rotation_meta_path.exists():
                try:
                    with open(rotation_meta_path, 'r', encoding='utf-8') as f:
                        rotation_metadata = json.load(f)
                    self.log(self.translator.get('log.zip_loaded_rotation_meta', '📄 Loaded rotation metadata'))
                except Exception as e:
                    self.log(self.translator.get('log.zip_error_reading_rotation', '⚠️ Error reading rotation metadata: {error}').replace('{error}', str(e)))
            
            # Tìm tunnel direction JSON
            tunnel_dir_patterns = [
                merged_map_dir / "merge_all_hba_tunnel_direction.json",
                merged_map_dir / "merged_all_tunnel_direction.json",
            ]
            for tunnel_file in merged_map_dir.glob("*_tunnel_direction.json"):
                if tunnel_file not in tunnel_dir_patterns:
                    tunnel_dir_patterns.append(tunnel_file)
            for pattern in tunnel_dir_patterns:
                if pattern.exists():
                    try:
                        with open(pattern, 'r', encoding='utf-8') as f:
                            tunnel_direction_data = json.load(f)
                        self.log(self.translator.get('log.zip_loaded_tunnel_dir', '📄 Loaded tunnel direction data'))
                        break
                    except Exception as e:
                        self.log(self.translator.get('log.zip_error_reading_tunnel', '⚠️ Error reading tunnel direction: {error}').replace('{error}', str(e)))
            
            with zipfile.ZipFile(zip_path, 'w', zipfile.ZIP_DEFLATED) as zipf:
                # Zip các folders
                for folder_name in existing_folders:
                    folder_path = log_root / folder_name
                    if folder_path.exists():
                        for root, dirs, files in os.walk(folder_path):
                            for file in files:
                                file_path = Path(root) / file
                                arcname = Path(folder_name) / file_path.relative_to(folder_path)
                                zipf.write(file_path, arcname)
                                self.log(self.translator.get('log.zip_file_added', '   ✓ Added: {path}').replace('{path}', str(arcname)))
                
                # Zip file QR_detect.json nếu có
                if has_qr_file:
                    zipf.write(qr_detect_file, "QR_detect.json")
                    self.log(self.translator.get('log.zip_file_added', '   ✓ Added: {path}').replace('{path}', 'QR_detect.json'))
                
                # Tạo và thêm map_metadata.json
                map_metadata = {
                    "map_name": map_name or "Unknown",
                    "vehicle_id": vehicle_id or "Unknown",
                    "created_at": datetime.now().isoformat(),
                }
                
                if rotation_metadata:
                    rotation_info = rotation_metadata.get("rotation_info", {})
                    map_metadata["tunnel_rotation"] = {
                        "angle_rad": rotation_info.get("rotation_angle_rad"),
                        "angle_deg": rotation_info.get("rotation_angle_deg"),
                        "rotation_axis": rotation_info.get("rotation_axis"),
                        "source_file": "merged_all_rotated_rotation_metadata.json"
                    }
                
                if tunnel_direction_data:
                    tunnel_dir_info = tunnel_direction_data.get("tunnel_direction", {})
                    if tunnel_dir_info:
                        tunnel_source_file = None
                        for pattern in tunnel_dir_patterns:
                            if pattern.exists():
                                tunnel_source_file = pattern.name
                                break
                        if not tunnel_source_file:
                            tunnel_source_file = "merged_all_tunnel_direction.json"
                        
                        map_metadata["tunnel_direction"] = {
                            "direction": tunnel_dir_info.get("direction"),
                            "source_file": tunnel_source_file
                        }
                
                map_metadata["files"] = {}
                if (log_root / "hba_map" / self.config.hba_pcd_name).exists():
                    map_metadata["files"]["pcd_original"] = f"hba_map/{self.config.hba_pcd_name}"
                if (merged_map_dir / "merged_all_rotated.pcd").exists():
                    map_metadata["files"]["pcd_rotated"] = "merged_map/merged_all_rotated.pcd"
                floorplan_dir = log_root / "floorplan_2d"
                if floorplan_dir.exists():
                    top_png = list(floorplan_dir.glob("*_top.png"))
                    side_png = list(floorplan_dir.glob("*_side_x.png"))
                    if top_png:
                        map_metadata["files"]["floorplan_top"] = f"floorplan_2d/{top_png[0].name}"
                    if side_png:
                        map_metadata["files"]["floorplan_side"] = f"floorplan_2d/{side_png[0].name}"
                
                # Ghi map_metadata.json vào zip
                import io
                metadata_json_str = json.dumps(map_metadata, ensure_ascii=False, indent=2)
                zipf.writestr("map_metadata.json", metadata_json_str.encode('utf-8'))
                self.log(self.translator.get('log.zip_file_added', '   ✓ Added: {path}').replace('{path}', 'map_metadata.json'))
            
            if zip_path.exists():
                size_mb = zip_path.stat().st_size / (1024 * 1024)
                self.log("=" * 60)
                self.log(self.translator.get('log.zip_success', '✅ Zip file created successfully!'))
                self.log(self.translator.get('log.zip_file', '📁 File: {path}').replace('{path}', str(zip_path)))
                self.log(self.translator.get('log.zip_size', '💾 Size: {size:.2f} MB').replace('{size:.2f}', f'{size_mb:.2f}'))
                self.log("=" * 60)
                return zip_path
            else:
                self.log(self.translator.get('log.zip_not_created', '❌ Zip file was not created'))
                return None
                
        except Exception as e:
            self.log(self.translator.get('log.zip_error', '❌ Error creating zip file: {error}').replace('{error}', str(e)))
            import traceback
            self.log(f"   Details: {traceback.format_exc()}")
            return None
    
    def upload_map_to_backend(self, zip_path: Path, map_name: str, vehicle_id: str):
        """Upload file zip map lên backend với retry mechanism"""
        if not REQUESTS_AVAILABLE:
            msg = self.translator.get('log.upload_backend_requests_missing', 'Missing requests library. Install with: pip install requests')
            self.log(f"❌ {msg}")
            return False, msg
        
        zip_path = Path(zip_path)
        
        if not zip_path.exists():
            error_msg = self.translator.get('log.upload_backend_file_not_found', 'File not found: {path}').replace('{path}', str(zip_path))
            self.log(f"❌ {error_msg}")
            return False, error_msg
        
        try:
            file_size = zip_path.stat().st_size
            file_size_mb = file_size / (1024 * 1024)
            
            if file_size == 0:
                error_msg = self.translator.get('log.upload_file_empty', 'File is empty: {path}').replace('{path}', str(zip_path))
                self.log(f"❌ {error_msg}")
                return False, error_msg
            
            if file_size > 500 * 1024 * 1024:
                self.log(self.translator.get('log.upload_file_large', 
                    '⚠️ Warning: File is very large ({size:.1f}MB). Upload may take a long time.').replace('{size:.1f}', f'{file_size_mb:.1f}'))
            
            self.log(self.translator.get('log.upload_file_size', 
                '📦 File size: {size:.2f} MB').replace('{size:.2f}', f'{file_size_mb:.2f}'))
        except OSError as e:
            error_msg = self.translator.get('log.upload_cannot_read_file', 
                'Cannot read file: {error}').replace('{error}', str(e))
            self.log(f"❌ {error_msg}")
            return False, error_msg
        
        upload_url = f"{self.config.backend_base_url.rstrip('/')}/api/v1/maps/upload"
        self.log(self.translator.get('log.upload_endpoint', '🌐 Endpoint: {url}').replace('{url}', upload_url))
        self.log(f"📝 Metadata: map_name='{map_name}', vehicle_id='{vehicle_id}'")
        
        # Tính toán timeout động
        base_timeout = 60
        timeout_per_10mb = 30
        dynamic_timeout = base_timeout + int((file_size_mb / 10) * timeout_per_10mb)
        upload_timeout = min(dynamic_timeout, 600)
        self.log(self.translator.get('log.upload_timeout', 
            '⏱️ Upload timeout: {timeout}s').replace('{timeout}', str(upload_timeout)))
        
        # Retry configuration
        max_retries = 3
        retry_delays = [2, 5, 10]
        
        session = None
        try:
            session = requests.Session()
            adapter = requests.adapters.HTTPAdapter(
                pool_connections=1,
                pool_maxsize=1,
                max_retries=0
            )
            session.mount('http://', adapter)
            session.mount('https://', adapter)
            
            last_error = None
            for attempt in range(max_retries):
                if attempt > 0:
                    delay = retry_delays[min(attempt - 1, len(retry_delays) - 1)]
                    self.log(self.translator.get('log.upload_retry', 
                        '🔄 Retrying upload (attempt {attempt}/{max}) after {delay}s...').replace('{attempt}', str(attempt + 1)).replace('{max}', str(max_retries)).replace('{delay}', str(delay)))
                    time.sleep(delay)
                
                try:
                    self.log(self.translator.get('log.upload_starting', 
                        '📤 Starting upload (attempt {attempt}/{max})...').replace('{attempt}', str(attempt + 1)).replace('{max}', str(max_retries)))
                    
                    start_time = time.time()
                    
                    with open(zip_path, "rb") as f:
                        files = {"file": (zip_path.name, f, "application/zip")}
                        
                        response = session.post(
                            upload_url,
                            data={
                                "map_name": map_name,
                                "vehicle_id": vehicle_id
                            },
                            files=files,
                            timeout=(10, upload_timeout),
                            stream=False
                        )
                    
                    upload_duration = time.time() - start_time
                    upload_speed_mbps = (file_size_mb / upload_duration) if upload_duration > 0 else 0
                    
                    self.log(self.translator.get('log.upload_completed', 
                        '✅ Upload completed in {duration:.1f}s ({speed:.2f} MB/s)').replace('{duration:.1f}', f'{upload_duration:.1f}').replace('{speed:.2f}', f'{upload_speed_mbps:.2f}'))
                    
                    if response.status_code in (200, 201):
                        upload_id = None
                        try:
                            data = response.json()
                            upload_id = data.get("upload_id") or data.get("uploadId") or data.get("id")
                            
                            if upload_id:
                                self.log(self.translator.get('log.upload_success_with_id', 
                                    '✅ Upload successful! Upload ID: {id}').replace('{id}', str(upload_id)))
                            else:
                                self.log(self.translator.get('log.upload_success_no_id', 
                                    '✅ Upload successful! (No upload ID in response)'))
                        except (ValueError, KeyError) as e:
                            self.log(self.translator.get('log.upload_success_invalid_response', 
                                '⚠️ Upload successful but response format unexpected: {error}').replace('{error}', str(e)))
                            upload_id = None
                        
                        return True, upload_id
                    
                    if 400 <= response.status_code < 500:
                        error_text = response.text.strip()[:500] if response.text else f"HTTP {response.status_code}"
                        error_msg = self.translator.get('log.upload_client_error', 
                            '❌ Client error (HTTP {code}): {error}').replace('{code}', str(response.status_code)).replace('{error}', error_text)
                        self.log(error_msg)
                        return False, error_text
                    
                    if 500 <= response.status_code < 600:
                        error_text = response.text.strip()[:500] if response.text else f"HTTP {response.status_code}"
                        last_error = f"Server error (HTTP {response.status_code}): {error_text}"
                        self.log(f"⚠️ {last_error}")
                        continue
                    
                    error_text = response.text.strip()[:500] if response.text else f"HTTP {response.status_code}"
                    last_error = f"Unexpected status code {response.status_code}: {error_text}"
                    self.log(f"⚠️ {last_error}")
                    continue
                
                except requests.exceptions.Timeout as e:
                    last_error = self.translator.get('log.upload_timeout_error', 
                        'Upload timeout after {timeout}s').replace('{timeout}', str(upload_timeout))
                    self.log(f"⚠️ {last_error}")
                    if attempt < max_retries - 1:
                        continue
                    else:
                        return False, last_error
                
                except requests.exceptions.ConnectionError as e:
                    last_error = self.translator.get('log.upload_connection_error', 
                        'Connection error: {error}').replace('{error}', str(e))
                    self.log(f"⚠️ {last_error}")
                    if attempt < max_retries - 1:
                        continue
                    else:
                        return False, last_error
                
                except requests.exceptions.RequestException as e:
                    last_error = self.translator.get('log.upload_request_error', 
                        'Request error: {error}').replace('{error}', str(e))
                    self.log(f"⚠️ {last_error}")
                    if attempt < max_retries - 1:
                        continue
                    else:
                        return False, last_error
                
                except Exception as e:
                    error_msg = self.translator.get('log.upload_unexpected_error', 
                        'Unexpected error: {error}').replace('{error}', str(e))
                    self.log(f"❌ {error_msg}")
                    if attempt < max_retries - 1:
                        continue
                    else:
                        return False, error_msg
            
            return False, last_error or "Max retries exceeded"
            
        finally:
            if session:
                session.close()
