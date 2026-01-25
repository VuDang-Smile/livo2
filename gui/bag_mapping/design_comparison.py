"""
Design Map Comparison Module
Chứa logic so sánh bản đồ thiết kế với bản đồ được tạo ra
"""
import json
from pathlib import Path
from tkinter import filedialog, messagebox
from datetime import datetime
from typing import Tuple

# Import comparison utils
COMPARISON_UTILS_AVAILABLE = False
try:
    from pathlib import Path as PathLib
    scripts_dir = Path(__file__).parent.parent.parent / "scripts"
    import sys
    if str(scripts_dir) not in sys.path:
        sys.path.insert(0, str(scripts_dir))

    from pcd_comparison_utils import (
        load_pcd,
        load_obj_as_pointcloud,
        preprocess_pcd,
        compute_similarity_metrics,
        calculate_similarity_percentage,
        detect_drift,
        DEFAULT_ICP_MAX_ITERATIONS,
        DEFAULT_ICP_THRESHOLD_MULTIPLIER,
        get_principal_axis_from_pcd,
        calculate_pcd_length_along_principal_axis,
    )
    COMPARISON_UTILS_AVAILABLE = True
except (ImportError, SystemError, OSError, SystemExit) as e:
    COMPARISON_UTILS_AVAILABLE = False
    print(f"Warning: pcd_comparison_utils not available: {e}. Design map comparison will be disabled.")


class DesignMapComparison:
    """Quản lý design map comparison"""
    
    def __init__(self, config, translator, logger_callback, ui_update_callback=None):
        """
        Initialize design map comparison
        
        Args:
            config: BagMappingConfig instance
            translator: Translator instance
            logger_callback: Function để log messages
            ui_update_callback: Function để update UI (optional)
        """
        self.config = config
        self.translator = translator
        self.log = logger_callback
        self.ui_update = ui_update_callback
        
        self.design_file_path = None
        self.design_file_type = config.design_file_type
        self.comparison_result = None
    
    def update_design_type(self, design_type_var, comparison_status_label=None):
        """Cập nhật loại file thiết kế (PCD / OBJ) khi người dùng chọn"""
        self.design_file_type = design_type_var.get()
        if self.design_file_type == "obj":
            msg = self.translator.get('label.design_type_obj', 
                'Design type: OBJ | voxel=0.15 | sampling=uniform | Match PCD density + Save converted PCD')
        else:
            msg = self.translator.get('label.design_type_pcd', 'Design type: PCD')
        
        if comparison_status_label:
            comparison_status_label.config(text=msg, fg="blue", font=self.config.font_large)
        
        self.log(self.translator.get('log.design_type_set', 
            'CONFIG: Design map type set to {type}').replace('{type}', self.design_file_type.upper()))
        
        # Lưu lại config nếu đã có design file path
        if self.design_file_path:
            self._save_design_file_config(self.design_file_path, self.design_file_type)
    
    def browse_design_file(self, design_path_var, workspace_path, comparison_status_label=None):
        """Chọn file bản thiết kế (PCD hoặc OBJ) để so sánh"""
        # Thư mục gợi ý: dùng đường dẫn hiện tại hoặc Log
        initial_dir = design_path_var.get()
        if not initial_dir:
            initial_dir = str(workspace_path / "src" / "FAST-LIVO2" / "Log")

        if self.design_file_type == "obj":
            filetypes = self.config.obj_file_types
            title = self.translator.get('dialog.choose_design_obj_file', 'Choose Design OBJ File')
        else:
            filetypes = self.config.pcd_file_types
            title = self.translator.get('dialog.choose_design_pcd_file', 'Choose Design PCD File')

        filename = filedialog.askopenfilename(
            title=title,
            initialdir=initial_dir,
            filetypes=filetypes,
        )

        if filename:
            design_path_var.set(filename)
            self.design_file_path = filename
            file_ext = "OBJ" if self.design_file_type == "obj" else "PCD"
            self.log(self.translator.get('log.design_file_selected', 
                '✅ Design {file_ext} selected: {filename}').replace('{file_ext}', file_ext).replace('{filename}', Path(filename).name))
            
            if comparison_status_label:
                status_text = self.translator.get('label.design_file_selected', 
                    'Design file: {filename}').replace('{filename}', Path(filename).name)
                comparison_status_label.config(
                    text=status_text,
                    fg="green",
                    font=self.config.font_large,
                )
            
            # Lưu đường dẫn vào config để dùng lần sau
            self._save_design_file_config(filename, self.design_file_type)
    
    def run_design_map_comparison(
        self,
        generated_pcd_path: Path,
        comparison_status_label=None,
        root=None,
        tunnel_entrance_coords: Tuple[float, float, float] = None
    ):
        """
        So sánh bản đồ tạo ra với bản thiết kế
        
        Args:
            generated_pcd_path: Path đến generated PCD file
            comparison_status_label: Tkinter Label để hiển thị kết quả (optional)
            root: Tkinter root window để update UI (optional)
            
        Returns:
            (success: bool, similarity: float)
        """
        if not COMPARISON_UTILS_AVAILABLE:
            self.log("⚠️ Design comparison utils not available. Skipping comparison step.")
            return True, 100.0

        try:
            design_path_str = self.design_file_path
            if not design_path_str:
                self.log("⚠️ No design file selected. Skipping comparison.")
                return True, 100.0

            design_path = Path(design_path_str)
            if not design_path.exists():
                msg = f"Design file not found: {design_path}"
                self.log(f"❌ {msg}")
                messagebox.showerror("Design map comparison", msg)
                return False, 0.0

            if not generated_pcd_path.exists():
                msg = f"Generated map PCD not found: {generated_pcd_path}"
                self.log(f"❌ {msg}")
                messagebox.showerror(
                    self.translator.get('dialog.design_map_comparison', 'Design map comparison'), 
                    msg
                )
                return False, 0.0

            self.log(self.translator.get('log.running_design_map_comparison', 
                '🔍 Running design map comparison...'))
            self.log(f"   Design file: {design_path.name} ({self.design_file_type.upper()})")
            self.log(f"   Generated map: {generated_pcd_path.name}")

            # Load test PCD (bản đồ tạo ra)
            pcd_test = load_pcd(generated_pcd_path)
            test_points = len(pcd_test.points)
            
            # Calculate mapped PCD length and principal axis
            mapped_length = calculate_pcd_length_along_principal_axis(pcd_test)
            test_principal_direction, test_centroid = get_principal_axis_from_pcd(pcd_test)
            
            self.log(f"   Mapped PCD length: {mapped_length:.2f} m")
            self.log(f"   Mapped PCD principal direction: [{test_principal_direction[0]:.3f}, {test_principal_direction[1]:.3f}, {test_principal_direction[2]:.3f}]")
            
            # Prepare cropping parameters if tunnel entrance coordinates provided
            crop_params = None
            design_length_before = None
            design_length_after = None
            principal_direction = test_principal_direction
            
            if tunnel_entrance_coords is not None:
                self.log(f"   Tunnel entrance coordinates: ({tunnel_entrance_coords[0]:.3f}, {tunnel_entrance_coords[1]:.3f}, {tunnel_entrance_coords[2]:.3f})")
                
                # For design file, we'll use the same principal direction as test PCD
                # (assuming they should be aligned)
                principal_direction = test_principal_direction
                
                crop_params = {
                    "origin": tunnel_entrance_coords,
                    "direction": principal_direction.tolist(),
                    "max_length": mapped_length
                }
                
                self.log(f"   Will crop design file to length: {mapped_length:.2f} m")

            # Load reference từ PCD hoặc OBJ (with cropping if needed)
            if self.design_file_type == "obj":
                # Thiết lập sampling cho OBJ: uniform, match density, lưu PCD convert
                match_density = test_points if test_points > 0 else None
                self.log(
                    f"   OBJ settings: voxel=0.15, sampling=uniform, "
                    f"match_density={match_density}, save_converted_pcd=True"
                )

                # Tạo thư mục lưu PCD convert từ OBJ
                self.config.obj_converted_dir.mkdir(parents=True, exist_ok=True)
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                converted_pcd_path = self.config.obj_converted_dir / f"{design_path.stem}_converted_{timestamp}.pcd"

                # Convert OBJ -> point cloud (without cropping first to measure length)
                pcd_ref_temp = load_obj_as_pointcloud(
                    design_path,
                    sampling_method="uniform",
                    num_points=None,
                    match_density=match_density,
                )
                design_length_before = calculate_pcd_length_along_principal_axis(pcd_ref_temp)
                self.log(f"   Design PCD length (before cropping): {design_length_before:.2f} m")
                
                # Now convert with cropping if needed
                pcd_ref = load_obj_as_pointcloud(
                    design_path,
                    sampling_method="uniform",
                    num_points=None,
                    match_density=match_density,
                    crop_params=crop_params
                )
                design_length_after = calculate_pcd_length_along_principal_axis(pcd_ref) if crop_params else design_length_before
                self.log(f"   Converted OBJ to PCD with {len(pcd_ref.points):,} points")
                if crop_params:
                    self.log(f"   Design PCD length (after cropping): {design_length_after:.2f} m")
            else:
                converted_pcd_path = None
                # Load PCD without cropping first to measure length
                pcd_ref_temp = load_pcd(design_path)
                design_length_before = calculate_pcd_length_along_principal_axis(pcd_ref_temp)
                self.log(f"   Design PCD length (before cropping): {design_length_before:.2f} m")
                
                # Now load with cropping if needed
                pcd_ref = load_pcd(design_path, crop_params=crop_params)
                design_length_after = calculate_pcd_length_along_principal_axis(pcd_ref) if crop_params else design_length_before
                if crop_params:
                    self.log(f"   Design PCD length (after cropping): {design_length_after:.2f} m")

            ref_points = len(pcd_ref.points)
            self.log(f"   Reference points: {ref_points:,}")
            self.log(f"   Test points: {test_points:,}")

            # Preprocess với voxel size = 0.15, remove outliers
            voxel_size = 0.15
            self.log(f"   Preprocessing with voxel size = {voxel_size}")

            pcd_ref = preprocess_pcd(
                pcd_ref,
                voxel_size=voxel_size,
                remove_outliers=True,
            )
            pcd_test = preprocess_pcd(
                pcd_test,
                voxel_size=voxel_size,
                remove_outliers=True,
            )

            self.log(f"   After preprocessing - Reference: {len(pcd_ref.points):,} points")
            self.log(f"   After preprocessing - Test: {len(pcd_test.points):,} points")

            # ICP threshold tự động theo voxel size
            icp_threshold = voxel_size * DEFAULT_ICP_THRESHOLD_MULTIPLIER
            self.log(f"   ICP threshold: {icp_threshold:.3f} m")

            # Nếu là OBJ, lưu PCD convert vào thư mục riêng
            save_pcd_path = converted_pcd_path if converted_pcd_path is not None else None
            if save_pcd_path is not None:
                self.log(f"   Will save converted design PCD to: {save_pcd_path}")

            metrics = compute_similarity_metrics(
                pcd_ref,
                pcd_test,
                perform_icp=True,
                icp_threshold=icp_threshold,
                icp_max_iterations=DEFAULT_ICP_MAX_ITERATIONS,
                save_obj_pcd=save_pcd_path,
            )

            if save_pcd_path is not None:
                self.log(f"✅ Converted design PCD saved to: {save_pcd_path}")

            similarity = calculate_similarity_percentage(metrics)
            drift_detected, drift_reasons = detect_drift(metrics, similarity)

            # Lưu kết quả vào state
            self.comparison_result = {
                "design_file": str(design_path),
                "generated_file": str(generated_pcd_path),
                "metrics": metrics,
                "similarity": similarity,
                "drift_detected": drift_detected,
                "drift_reasons": drift_reasons,
            }
            
            # Add cropping info if cropping was performed
            if tunnel_entrance_coords is not None and crop_params is not None:
                self.comparison_result["cropping_info"] = {
                    "was_cropped": True,
                    "tunnel_entrance_coords": tunnel_entrance_coords,
                    "mapped_pcd_length": mapped_length,
                    "design_pcd_length_before": design_length_before,
                    "design_pcd_length_after": design_length_after,
                    "principal_axis_direction": principal_direction.tolist(),
                }

            # Log tóm tắt
            self.log("=" * 60)
            self.log(self.translator.get('log.design_map_comparison_results', 
                'Design Map Comparison Results'))
            
            # Log cropping info if available
            if tunnel_entrance_coords is not None and crop_params is not None:
                self.log(f"   Cropping Info:")
                self.log(f"     Tunnel entrance: ({tunnel_entrance_coords[0]:.3f}, {tunnel_entrance_coords[1]:.3f}, {tunnel_entrance_coords[2]:.3f})")
                self.log(f"     Mapped PCD length: {mapped_length:.2f} m")
                self.log(f"     Design PCD length (before): {design_length_before:.2f} m")
                self.log(f"     Design PCD length (after): {design_length_after:.2f} m")
                self.log(f"     Principal axis direction: [{principal_direction[0]:.3f}, {principal_direction[1]:.3f}, {principal_direction[2]:.3f}]")
            
            self.log(self.translator.get('label.similarity_percent', 
                '   Similarity: {similarity}%').replace('{similarity}', f'{similarity:.2f}'))
            self.log(f"   ICP fitness: {metrics.get('icp_fitness', 0.0):.4f}")
            self.log(f"   Hausdorff distance: {metrics.get('hausdorff_distance', 0.0):.4f} m")
            self.log(f"   Chamfer distance: {metrics.get('chamfer_distance', 0.0):.4f} m")
            self.log(f"   BBox overlap: {metrics.get('bbox_overlap', 0.0):.4f}")
            self.log(f"   Centroid distance: {metrics.get('centroid_distance', 0.0):.4f} m")
            self.log(f"   Density ratio: {metrics.get('density_ratio', 0.0):.4f}")

            status_text = self.translator.get('label.similarity_percent', 
                'Similarity: {similarity}%').replace('{similarity}', f'{similarity:.2f}')
            if drift_detected:
                self.log("   Drift detected:")
                for reason in drift_reasons:
                    self.log(f"     - {reason}")
                status_text += " (" + self.translator.get('label.possible_drift', 'Possible drift') + ")"
            
            # Xác định màu sắc dựa trên % similarity
            if similarity >= 90:
                status_color = "green"
                if not drift_detected:
                    status_text = self.translator.get('label.similarity_percent', 
                        'Similarity: {similarity}%').replace('{similarity}', f'{similarity:.2f}') + \
                        " (" + self.translator.get('label.no_significant_drift', 'No significant drift') + ")"
            elif similarity >= 50:
                status_color = "orange"
                if not drift_detected:
                    status_text = self.translator.get('label.similarity_percent', 
                        'Similarity: {similarity}%').replace('{similarity}', f'{similarity:.2f}') + \
                        " (" + self.translator.get('label.moderate_match', 'Moderate match') + ")"
            else:
                status_color = "red"
                if not drift_detected:
                    status_text = self.translator.get('label.similarity_percent', 
                        'Similarity: {similarity}%').replace('{similarity}', f'{similarity:.2f}') + \
                        " (" + self.translator.get('label.low_match', 'Low match') + ")"

            self.log("=" * 60)

            # Cập nhật label UI trong main thread
            if comparison_status_label and root:
                root.after(
                    0,
                    lambda txt=status_text, col=status_color: comparison_status_label.config(
                        text=txt, fg=col, font=self.config.font_large
                    ),
                )

            return True, similarity

        except Exception as e:
            self.log(f"❌ Error in design map comparison: {e}")
            import traceback
            self.log(f"   Details: {traceback.format_exc()[:500]}")
            try:
                messagebox.showerror(
                    self.translator.get('dialog.error', 'Error'),
                    f"Error in design map comparison: {e}"
                )
            except:
                pass
            return False, 0.0
    
    def load_design_file_config(self, design_path_var=None, design_type_var=None, comparison_status_label=None):
        """Tải đường dẫn design file đã lưu từ config"""
        try:
            if not self.config.design_config_path.exists():
                return
            
            with open(self.config.design_config_path, 'r', encoding='utf-8') as f:
                config_data = json.load(f)
            
            design_path = config_data.get('design_file_path')
            design_type = config_data.get('design_file_type', 'pcd')
            
            # Kiểm tra file có tồn tại không
            if design_path and Path(design_path).exists():
                self.design_file_path = design_path
                self.design_file_type = design_type
                
                # Cập nhật UI nếu đã được khởi tạo
                if design_path_var:
                    design_path_var.set(design_path)
                
                if design_type_var:
                    design_type_var.set(design_type)
                
                # Cập nhật status label
                if comparison_status_label:
                    file_name = Path(design_path).name
                    status_text = self.translator.get('label.design_file_selected', 
                        'Design file: {filename}').replace('{filename}', file_name)
                    comparison_status_label.config(
                        text=status_text,
                        fg="green",
                        font=self.config.font_large,
                    )
                
                self.log(self.translator.get('log.design_file_loaded_from_config', 
                    '✅ Design file loaded from config: {filename}').replace('{filename}', Path(design_path).name))
            else:
                # File không tồn tại, xóa config cũ
                if design_path:
                    self.log(self.translator.get('log.design_file_not_found_removing_config', 
                        '⚠️ Saved design file not found, removing from config: {path}').replace('{path}', design_path))
                    self._save_design_file_config(None, None)
                    
        except json.JSONDecodeError as e:
            self.log(self.translator.get('log.design_config_invalid_json', 
                '⚠️ Invalid design config JSON, ignoring: {error}').replace('{error}', str(e)))
        except Exception as e:
            self.log(self.translator.get('log.design_config_load_error', 
                '⚠️ Error loading design config: {error}').replace('{error}', str(e)))
    
    def _save_design_file_config(self, file_path, file_type):
        """Lưu đường dẫn design file vào config"""
        try:
            config = {}
            
            if file_path and Path(file_path).exists():
                config['design_file_path'] = str(file_path)
                config['design_file_type'] = file_type or self.design_file_type
            else:
                # Nếu file_path là None, xóa config
                if self.config.design_config_path.exists():
                    self.config.design_config_path.unlink()
                return
            
            with open(self.config.design_config_path, 'w', encoding='utf-8') as f:
                json.dump(config, f, indent=2, ensure_ascii=False)
            
        except Exception as e:
            self.log(self.translator.get('log.design_config_save_error', 
                '⚠️ Error saving design config: {error}').replace('{error}', str(e)))
