"""
PCD Processing Module
Chứa logic xử lý PCD files: merge, HBA, SC, floorplan generation
"""
import subprocess
import shutil
import time
import os
import signal
from pathlib import Path
from tkinter import messagebox


class PCDProcessor:
    """Quản lý xử lý PCD files"""
    
    def __init__(self, config, translator, logger_callback):
        """
        Initialize PCD processor
        
        Args:
            config: BagMappingConfig instance
            translator: Translator instance
            logger_callback: Function để log messages
        """
        self.config = config
        self.translator = translator
        self.log = logger_callback
    
    def merge_pcd_files(self, silent=False):
        """Gộp tất cả các file PCD trong thư mục Log/PCD thành một file lớn"""
        pcd_dir = self.config.pcd_dir
        
        if not pcd_dir.exists():
            if not silent:
                messagebox.showerror(
                    self.translator.get('dialog.error', 'Error'),
                    self.translator.get('dialog.pcd_dir_not_exists', 'PCD directory does not exist: {path}').replace('{path}', str(pcd_dir))
                )
            self.log(self.translator.get('log.pcd_dir_not_exists', '❌ PCD directory does not exist: {path}').replace('{path}', str(pcd_dir)))
            return False
        
        # Tìm tất cả file PCD (bỏ qua file merged nếu có)
        pcd_files = sorted([f for f in pcd_dir.glob(self.config.pcd_file_pattern) if f.name != self.config.merged_pcd_name])
        
        if not pcd_files:
            if not silent:
                messagebox.showwarning(
                    self.translator.get('dialog.warning', 'Warning'),
                    self.translator.get('dialog.no_pcd_files_warning', 'No PCD files found in directory: {path}').replace('{path}', str(pcd_dir))
                )
            self.log(self.translator.get('log.no_pcd_files', '⚠️ No PCD files found in: {path}').replace('{path}', str(pcd_dir)))
            return False
        
        self.log("=" * 60)
        self.log(self.translator.get('log.merge_start', '🔗 Starting to merge {count} PCD files...').replace('{count}', str(len(pcd_files))))
        self.log(self.translator.get('log.merge_directory', '📁 Directory: {path}').replace('{path}', str(pcd_dir)))
        
        try:
            # Kiểm tra open3d
            try:
                import open3d as o3d
            except ImportError:
                error_msg = self.translator.get('dialog.open3d_not_installed', 
                    'open3d library is not installed.\n\nPlease install with:\npip install open3d\n\nOr add to requirements.txt and reinstall.')
                if not silent:
                    messagebox.showerror(
                        self.translator.get('dialog.library_missing', 'Missing Library'),
                        error_msg
                    )
                self.log(self.translator.get('log.open3d_not_installed', '❌ open3d is not installed. Please install: pip install open3d'))
                return False
            
            merged_cloud = None
            total_points = 0
            
            # Sử dụng open3d để merge
            for i, pcd_file in enumerate(pcd_files, 1):
                try:
                    self.log(self.translator.get('log.reading_pcd_file', '📖 Reading file {current}/{total}: {filename}').replace('{current}', str(i)).replace('{total}', str(len(pcd_files))).replace('{filename}', pcd_file.name))
                    cloud = o3d.io.read_point_cloud(str(pcd_file))
                    
                    if len(cloud.points) == 0:
                        self.log(self.translator.get('log.pcd_file_empty', '⚠️ File {filename} is empty, skipping').replace('{filename}', pcd_file.name))
                        continue
                    
                    if merged_cloud is None:
                        merged_cloud = cloud
                    else:
                        merged_cloud += cloud
                    
                    total_points += len(cloud.points)
                    self.log(self.translator.get('log.points_added', '   ✓ Added {count:,} points from {filename}').replace('{count:,}', f'{len(cloud.points):,}').replace('{filename}', pcd_file.name))
                    
                except Exception as e:
                    self.log(self.translator.get('log.error_reading_pcd', '❌ Error reading {filename}: {error}').replace('{filename}', pcd_file.name).replace('{error}', str(e)))
                    continue
            
            if merged_cloud is None or len(merged_cloud.points) == 0:
                if not silent:
                    messagebox.showerror(
                        self.translator.get('dialog.error', 'Error'),
                        self.translator.get('dialog.merge_no_valid_points', 'Cannot merge PCD files. No valid points.')
                    )
                self.log(self.translator.get('log.no_points_to_merge', '❌ No points to merge'))
                return False
            
            # Tạo folder riêng cho merged map
            merged_map_dir = self.config.merged_map_dir
            merged_map_dir.mkdir(parents=True, exist_ok=True)
            
            # Lưu file merged vào folder riêng
            output_file = merged_map_dir / self.config.merged_pcd_name
            self.log(self.translator.get('log.saving_merged_file', '💾 Saving merged file: {filename}').replace('{filename}', output_file.name))
            self.log(f"📁 Output folder: {merged_map_dir}")
            o3d.io.write_point_cloud(str(output_file), merged_cloud)
            
            # Kiểm tra kết quả
            if output_file.exists():
                size_mb = output_file.stat().st_size / (1024 * 1024)
                self.log("=" * 60)
                self.log(self.translator.get('log.merge_success', '✅ Merge PCD successful!'))
                self.log(self.translator.get('log.merge_output_file', '📁 Output file: {filename}').replace('{filename}', output_file.name))
                self.log(self.translator.get('log.merge_total_points', '📊 Total points: {count:,}').replace('{count:,}', f'{total_points:,}'))
                self.log(self.translator.get('log.merge_file_size', '💾 File size: {size:.2f} MB').replace('{size:.2f}', f'{size_mb:.2f}'))
                self.log("=" * 60)
                if not silent:
                    messagebox.showinfo(
                        self.translator.get('dialog.merge_success_title', 'Success'),
                        self.translator.get('dialog.merge_success_message', 
                            'Successfully merged {count} PCD files!\n\nOutput file: {filename}\nTotal points: {points:,}\nFile size: {size:.2f} MB')
                            .replace('{count}', str(len(pcd_files)))
                            .replace('{filename}', output_file.name)
                            .replace('{points:,}', f'{total_points:,}')
                            .replace('{size:.2f}', f'{size_mb:.2f}')
                    )
                return True
            else:
                raise Exception("File output không được tạo")
                
        except Exception as e:
            error_msg = f"Error merging PCD files: {e}"
            self.log(f"❌ {error_msg}")
            if not silent:
                messagebox.showerror(
                    self.translator.get('dialog.error', 'Error'),
                    error_msg
                )
            return False
    
    def run_hba_core(self):
        """Core logic của HBA optimization"""
        pcd_dir = self.config.pcd_dir
        merged_map_dir = self.config.merged_map_dir
        hba_map_dir = self.config.hba_map_dir
        hba_script = self.config.hba_script
        
        if not hba_script.exists():
            self.log(self.translator.get('log.hba_script_not_found', '❌ HBA script not found at: {path}').replace('{path}', str(hba_script)))
            return False

        # scans_pos.json luôn nằm trong PCD gốc
        pose_file = pcd_dir / self.config.scans_pos_file
        if not pose_file.exists():
            self.log(self.translator.get('log.scans_pos_missing', '❌ Missing scans_pos.json in {path}').replace('{path}', str(pcd_dir)))
            return False

        # Kiểm tra input: ưu tiên merged_map, fallback về PCD
        merged_pcd = merged_map_dir / self.config.merged_pcd_name
        if merged_pcd.exists():
            input_dir = merged_map_dir
            input_pcd = merged_pcd
        else:
            # Nếu không có merged_map, tìm file PCD đầu tiên trong PCD gốc
            pcd_files = sorted([f for f in pcd_dir.glob(self.config.pcd_file_pattern) if f.name not in [self.config.merged_pcd_name, self.config.hba_pcd_name]])
            if not pcd_files:
                self.log(self.translator.get('log.no_pcd_input', '❌ No PCD input files found'))
                return False
            input_dir = pcd_dir
            input_pcd = pcd_files[0]
        
        if not input_dir.exists():
            self.log(self.translator.get('log.input_dir_not_exists', '❌ Input directory does not exist: {path}').replace('{path}', str(input_dir)))
            return False

        # Tạo folder riêng cho HBA output
        hba_map_dir.mkdir(parents=True, exist_ok=True)
        
        self.log("=" * 60)
        self.log(self.translator.get('log.hba_start', '🪄 Starting map optimization with HBA Standalone...'))
        self.log(self.translator.get('log.hba_input', '📁 Input: {path}').replace('{path}', str(input_dir)))
        self.log(f"📁 Input PCD: {input_pcd.name}")
        self.log(self.translator.get('log.hba_output', '📁 Output: {path}').replace('{path}', str(hba_map_dir)))

        try:
            # Copy scans_pos.json vào input_dir nếu chưa có
            input_pose_file = input_dir / self.config.scans_pos_file
            pose_file_copied = False
            if not input_pose_file.exists() and pose_file.exists():
                shutil.copy2(pose_file, input_pose_file)
                pose_file_copied = True
                self.log(self.translator.get('log.copied_scans_pos', '📋 Copied scans_pos.json to {path}').replace('{path}', str(input_dir)))
            
            # Chạy HBA với input_dir
            cmd = f"python3 {hba_script} --input_dir {input_dir}"
            process = subprocess.Popen(
                cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True
            )
            
            for line in iter(process.stdout.readline, ''):
                if line:
                    self.log(f"[HBA] {line.strip()}")
            
            process.wait()
            
            if process.returncode == 0:
                # Script HBA tạo file merge_all_hba.pcd trong input_dir, di chuyển vào hba_map_dir
                source_file = input_dir / self.config.hba_pcd_name
                if source_file.exists():
                    # Di chuyển file output vào folder riêng
                    output_file = hba_map_dir / self.config.hba_pcd_name
                    shutil.move(source_file, output_file)
                    self.log(self.translator.get('log.moved_hba_file', '📦 Moved merge_all_hba.pcd from {from} to {to}').replace('{from}', str(input_dir)).replace('{to}', str(hba_map_dir)))
                    
                    # Copy scans_pos.json vào hba_map_dir
                    if input_pose_file.exists():
                        shutil.copy2(input_pose_file, hba_map_dir / self.config.scans_pos_file)
                    
                    # Xóa scans_pos.json khỏi merged_map nếu đã copy tạm
                    if input_dir == merged_map_dir and pose_file_copied and input_pose_file.exists():
                        try:
                            input_pose_file.unlink()
                            self.log(self.translator.get('log.removed_temp_scans_pos', '🧹 Removed temporary scans_pos.json from {path}').replace('{path}', str(merged_map_dir)))
                        except Exception as e:
                            self.log(self.translator.get('log.cannot_remove_temp', '⚠️ Cannot remove temporary scans_pos.json: {error}').replace('{error}', str(e)))
                    
                    self.log(self.translator.get('log.hba_complete', '✅ HBA Optimization complete!'))
                    self.log(f"📁 Output: {output_file}")
                    return True
                else:
                    self.log(self.translator.get('log.hba_no_output', '⚠️ HBA completed but output file not found.'))
                    return False
            else:
                self.log(self.translator.get('log.hba_failed', '❌ HBA failed with code {code}').replace('{code}', str(process.returncode)))
                return False
        except Exception as e:
            self.log(self.translator.get('log.hba_error', '❌ HBA error: {error}').replace('{error}', str(e)))
            import traceback
            self.log(f"   Details: {traceback.format_exc()}")
            return False
    
    def compute_tunnel_direction(self):
        """Tính vector hướng đường hầm từ PCD sau HBA"""
        hba_map_dir = self.config.hba_map_dir
        merged_map_dir = self.config.merged_map_dir
        compute_dir_script = self.config.compute_dir_script
        
        if not compute_dir_script.exists():
            self.log(self.translator.get('log.compute_dir_script_not_found', '❌ compute_tunnel_direction.py script not found: {path}').replace('{path}', str(compute_dir_script)))
            return False
        
        # Input PCD từ hba_map
        input_pcd = hba_map_dir / self.config.hba_pcd_name
        if not input_pcd.exists():
            self.log(self.translator.get('log.compute_dir_no_hba_pcd', '❌ merge_all_hba.pcd not found in hba_map directory'))
            return False
        
        # Output sẽ được lưu vào merged_map
        merged_map_dir.mkdir(parents=True, exist_ok=True)
        output_dir = merged_map_dir
        
        self.log("=" * 60)
        self.log(self.translator.get('log.compute_dir_start', '🧭 Computing tunnel direction vector...'))
        self.log(f"📁 Input PCD: {input_pcd}")
        self.log(f"📁 Output directory: {output_dir}")
        
        try:
            cmd = [
                "python3",
                str(compute_dir_script),
                "--pcd_path", str(input_pcd),
                "--output_dir", str(output_dir)
            ]
            
            process = subprocess.Popen(
                cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True
            )
            
            for line in iter(process.stdout.readline, ''):
                if line:
                    self.log(f"[TunnelDirection] {line.strip()}")
            
            process.wait()
            
            if process.returncode == 0:
                expected_output = output_dir / f"{input_pcd.stem}_tunnel_direction.json"
                if expected_output.exists():
                    self.log(self.translator.get('log.compute_dir_success', '✅ Tunnel direction computed successfully!'))
                    return True
                else:
                    self.log(self.translator.get('log.compute_dir_no_output', '⚠️ Script completed but output file not found'))
                    return False
            else:
                self.log(self.translator.get('log.compute_dir_failed', '❌ compute_tunnel_direction.py failed with code {code}').replace('{code}', str(process.returncode)))
                return False
                
        except Exception as e:
            self.log(self.translator.get('log.compute_dir_error', '❌ Error computing tunnel direction: {error}').replace('{error}', str(e)))
            return False
    
    def rotate_pcd_to_positive_x(self):
        """Xoay PCD về hướng +x dựa trên tunnel direction"""
        hba_map_dir = self.config.hba_map_dir
        merged_map_dir = self.config.merged_map_dir
        rotate_script = self.config.rotate_pcd_script
        
        if not rotate_script.exists():
            self.log(self.translator.get('log.rotate_script_not_found', '❌ rotate_pcd_to_positive_x.py script not found: {path}').replace('{path}', str(rotate_script)))
            return False
        
        # Input PCD từ hba_map
        input_pcd = hba_map_dir / self.config.hba_pcd_name
        if not input_pcd.exists():
            self.log(self.translator.get('log.rotate_no_hba_pcd', '❌ merge_all_hba.pcd not found in hba_map directory'))
            return False
        
        # Tìm direction JSON trong merged_map
        direction_json = merged_map_dir / f"{input_pcd.stem}_tunnel_direction.json"
        if not direction_json.exists():
            direction_json = merged_map_dir / "merged_all_tunnel_direction.json"
            if not direction_json.exists():
                self.log(self.translator.get('log.rotate_no_direction_json', '❌ Tunnel direction JSON not found'))
                return False
        
        output_dir = merged_map_dir
        
        self.log("=" * 60)
        self.log(self.translator.get('log.rotate_start', '🔄 Rotating PCD to align tunnel with +X axis...'))
        self.log(f"📁 Input PCD: {input_pcd}")
        self.log(f"📁 Direction JSON: {direction_json}")
        
        try:
            cmd = [
                "python3",
                str(rotate_script),
                "--pcd_path", str(input_pcd),
                "--direction_json", str(direction_json),
                "--output_dir", str(output_dir),
                "--output_name", "merged_all_rotated"
            ]
            
            process = subprocess.Popen(
                cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True
            )
            
            for line in iter(process.stdout.readline, ''):
                if line:
                    self.log(f"[RotatePCD] {line.strip()}")
            
            process.wait()
            
            if process.returncode == 0:
                expected_pcd = output_dir / "merged_all_rotated.pcd"
                expected_metadata = output_dir / "merged_all_rotated_rotation_metadata.json"
                
                if expected_pcd.exists() and expected_metadata.exists():
                    self.log(self.translator.get('log.rotate_success', '✅ PCD rotated successfully!'))
                    return True
                else:
                    self.log(self.translator.get('log.rotate_missing_output', '⚠️ Script completed but output files missing'))
                    return False
            else:
                self.log(self.translator.get('log.rotate_failed', '❌ rotate_pcd_to_positive_x.py failed with code {code}').replace('{code}', str(process.returncode)))
                return False
                
        except Exception as e:
            self.log(self.translator.get('log.rotate_error', '❌ Error rotating PCD: {error}').replace('{error}', str(e)))
            return False
    
    def run_sc_core(self):
        """Core logic của ScanContext tiling"""
        hba_map_dir = self.config.hba_map_dir
        merged_map_dir = self.config.merged_map_dir
        sc_script = self.config.sc_script
        output_dir = self.config.workspace_path / "src" / "FAST-LIVO2" / "Log" / "fastloc_map"

        if not sc_script.exists():
            self.log(self.translator.get('log.sc_script_not_found', '❌ ScanContext script not found: {path}').replace('{path}', str(sc_script)))
            return False

        # Ưu tiên input từ hba_map, fallback về merged_map
        input_pcd = hba_map_dir / self.config.hba_pcd_name
        if not input_pcd.exists():
            self.log(self.translator.get('log.hba_not_found_try_merged', '⚠️ HBA map not found, trying merged_all.pcd...'))
            input_pcd = merged_map_dir / self.config.merged_pcd_name
            if not input_pcd.exists():
                self.log(self.translator.get('log.no_merged_pcd_found', '❌ No merged PCD files found'))
                return False

        # Validate input file
        try:
            file_size = input_pcd.stat().st_size
            file_size_mb = file_size / (1024 * 1024)
            
            if file_size == 0:
                self.log(self.translator.get('log.sc_input_empty', '❌ Input PCD file is empty'))
                return False
            
            self.log(self.translator.get('log.sc_input_size', '📦 Input file size: {size:.2f} MB').replace('{size:.2f}', f'{file_size_mb:.2f}'))
        except OSError as e:
            self.log(self.translator.get('log.sc_cannot_read_input', '❌ Cannot read input file: {error}').replace('{error}', str(e)))
            return False

        self.log("=" * 60)
        self.log(self.translator.get('log.sc_start', '🗺️ Preparing ScanContext map from {filename}...').replace('{filename}', input_pcd.name))
        self.log(f"📁 Input: {input_pcd.parent}")
        self.log(f"📁 Output: {output_dir}")

        try:
            output_dir.mkdir(parents=True, exist_ok=True)
        except Exception as e:
            self.log(self.translator.get('log.sc_cannot_create_output', '❌ Cannot create output directory: {error}').replace('{error}', str(e)))
            return False

        process = None
        try:
            # Tính toán timeout động
            base_timeout = 300
            timeout_per_100mb = 60
            dynamic_timeout = base_timeout + int((file_size_mb / 100) * timeout_per_100mb)
            process_timeout = min(dynamic_timeout, 3600)
            
            self.log(self.translator.get('log.sc_timeout', '⏱️ Process timeout: {timeout}s').replace('{timeout}', str(process_timeout)))
            
            cmd = (
                f"python3 {sc_script} "
                f"--input_pcd {input_pcd} "
                f"--output_dir {output_dir} "
                f"--strip_color --voxel_size 0.2 --tile_size 50.0"
            )
            
            self.log(self.translator.get('log.sc_running', '▶️ Running ScanContext script...'))
            
            process = subprocess.Popen(
                cmd, 
                shell=True, 
                stdout=subprocess.PIPE, 
                stderr=subprocess.STDOUT, 
                text=True,
                bufsize=1,
                universal_newlines=True
            )
            
            # Đọc output với timeout
            output_lines = []
            start_time = time.time()
            
            while True:
                elapsed = time.time() - start_time
                if elapsed > process_timeout:
                    self.log(self.translator.get('log.sc_timeout_reached', '❌ Process timeout reached ({timeout}s). Terminating...').replace('{timeout}', str(process_timeout)))
                    try:
                        if hasattr(os, 'setsid'):
                            os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                        else:
                            process.terminate()
                    except:
                        pass
                    try:
                        process.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        try:
                            if hasattr(os, 'setsid'):
                                os.killpg(os.getpgid(process.pid), signal.SIGKILL)
                            else:
                                process.kill()
                        except:
                            pass
                    return False
                
                try:
                    line = process.stdout.readline()
                    if not line:
                        break
                    if line.strip():
                        output_lines.append(line.strip())
                        self.log(f"[ScanContext] {line.strip()}")
                except Exception as e:
                    self.log(f"⚠️ Error reading output: {e}")
                    break
            
            try:
                return_code = process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                self.log(self.translator.get('log.sc_process_hanging', '⚠️ Process seems to be hanging, forcing termination...'))
                try:
                    if hasattr(os, 'setsid'):
                        os.killpg(os.getpgid(process.pid), signal.SIGKILL)
                    else:
                        process.kill()
                except:
                    pass
                return False
            
            if return_code == 0:
                index_file = output_dir / "index.json"
                if not index_file.exists():
                    self.log(self.translator.get('log.sc_no_index_file', '⚠️ Process completed but index.json not found'))
                    return False
                
                tiles_dir = output_dir / "pcd"
                if tiles_dir.exists():
                    tile_files = list(tiles_dir.glob("*.pcd"))
                else:
                    tile_files = list(output_dir.glob("*.pcd"))
                
                if len(tile_files) == 0:
                    self.log(self.translator.get('log.sc_no_tiles', '⚠️ Process completed but no tile files found'))
                    return False
                
                self.log("=" * 60)
                self.log(self.translator.get('log.sc_success', '✅ ScanContext Tiling successful!'))
                self.log(f"📁 Output: {output_dir}")
                self.log(self.translator.get('log.sc_tiles_created', '📊 Tiles created: {count}').replace('{count}', str(len(tile_files))))
                self.log("=" * 60)
                return True
            else:
                self.log(self.translator.get('log.sc_failed', '❌ Tiling failed with code {code}').replace('{code}', str(return_code)))
                if output_lines:
                    self.log(self.translator.get('log.sc_last_output', 'Last output lines:'))
                    for line in output_lines[-10:]:
                        self.log(f"   {line}")
                return False
                
        except Exception as e:
            self.log(self.translator.get('log.sc_error', '❌ ScanContext error: {error}').replace('{error}', str(e)))
            if process:
                try:
                    if hasattr(os, 'setsid'):
                        os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                    else:
                        process.terminate()
                except:
                    pass
            return False
    
    def generate_floorplan(self):
        """Generate floorplan from PCD file"""
        merged_map_dir = self.config.merged_map_dir
        pcd_to_floorplan_script = self.config.pcd_to_floorplan_script
        
        # Use merged_all_rotated.pcd from merged_map directory
        input_pcd = merged_map_dir / "merged_all_rotated.pcd"
        if not input_pcd.exists():
            self.log(self.translator.get('log.floorplan_no_pcd_found', '❌ merged_all_rotated.pcd not found in merged_map directory'))
            return False
        
        output_dir = merged_map_dir / "floorplan"
        output_dir.mkdir(parents=True, exist_ok=True)
        
        self.log("=" * 60)
        self.log(self.translator.get('log.floorplan_start', '📐 Generating floorplan from PCD...'))
        self.log(f"📁 Input: {input_pcd}")
        self.log(f"📁 Output: {output_dir}")
        
        try:
            if not pcd_to_floorplan_script.exists():
                self.log(self.translator.get('log.floorplan_script_not_found', '❌ pcd_to_floorplan.py script not found'))
                return False
            
            cmd = [
                "python3",
                str(pcd_to_floorplan_script),
                "--pcd_path", str(input_pcd),
                "--output_dir", str(output_dir)
            ]
            
            process = subprocess.Popen(
                cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True
            )
            
            for line in iter(process.stdout.readline, ''):
                if line:
                    self.log(f"[Floorplan] {line.strip()}")
            
            process.wait()
            
            if process.returncode == 0:
                self.log(self.translator.get('log.floorplan_success', '✅ Floorplan generated successfully!'))
                return True
            else:
                self.log(self.translator.get('log.floorplan_failed', '❌ Floorplan generation failed with code {code}').replace('{code}', str(process.returncode)))
                return False
                
        except Exception as e:
            self.log(self.translator.get('log.floorplan_error', '❌ Error generating floorplan: {error}').replace('{error}', str(e)))
            return False
