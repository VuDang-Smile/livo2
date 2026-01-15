#!/usr/bin/env python3
"""
PCD Comparison Tab Module
Tab để so sánh 2 file PCD và phát hiện drift
"""

import threading
import json
from pathlib import Path
from datetime import datetime
import os
import sys
from functools import partial
from typing import Optional, Tuple

try:
    import tkinter as tk
    from tkinter import ttk, messagebox, scrolledtext, filedialog
except ImportError as e:
    print(f"Lỗi import: {e}")
    sys.exit(1)

# Import utilities
sys.path.insert(0, str(Path(__file__).parent.parent / "scripts"))
from pcd_comparison_utils import (
    load_pcd,
    load_obj_as_pointcloud,
    preprocess_pcd,
    compute_similarity_metrics,
    calculate_similarity_percentage,
    detect_drift,
    DEFAULT_VOXEL_SIZE,
    DEFAULT_ICP_MAX_ITERATIONS,
    DEFAULT_ICP_THRESHOLD_MULTIPLIER
)


class PCDComparisonTab(ttk.Frame):
    """Tab cho PCD Comparison"""
    
    def __init__(self, parent):
        super().__init__(parent)
        
        # Paths
        self.workspace_path = Path(__file__).parent.parent / "ws"
        self.fast_livo_path = self.workspace_path / "src" / "FAST-LIVO2"
        self.default_pcd_dir = self.fast_livo_path / "Log" / "PCD"
        
        # State
        self.is_comparing = False
        self.comparison_thread = None
        self.reference_pcd_path = None
        self.test_pcd_path = None
        self.last_results = None
        self.file_type = "pcd"  # "pcd" or "obj"
        
        # Tạo UI
        self.create_widgets()
    
    def create_widgets(self):
        """Tạo các widget cho tab PCD Comparison"""
        
        # Title
        title_label = ttk.Label(
            self,
            text="PCD Comparison Tool",
            font=("Arial", 16, "bold")
        )
        title_label.pack(pady=10)
        
        # Frame điều khiển
        control_frame = ttk.Frame(self, padding="10")
        control_frame.pack(fill=tk.X)
        
        # File type selection
        filetype_frame = ttk.LabelFrame(control_frame, text="File Type", padding="10")
        filetype_frame.pack(fill=tk.X, padx=10, pady=5)
        
        self.file_type_var = tk.StringVar(value="pcd")
        pcd_radio = ttk.Radiobutton(
            filetype_frame,
            text="PCD vs PCD",
            variable=self.file_type_var,
            value="pcd",
            command=self.on_file_type_changed
        )
        pcd_radio.pack(side=tk.LEFT, padx=10)
        
        obj_radio = ttk.Radiobutton(
            filetype_frame,
            text="OBJ vs PCD",
            variable=self.file_type_var,
            value="obj",
            command=self.on_file_type_changed
        )
        obj_radio.pack(side=tk.LEFT, padx=10)
        
        # Frame chọn Reference
        self.ref_frame = ttk.LabelFrame(control_frame, text="Reference PCD (Chuẩn)", padding="10")
        self.ref_frame.pack(fill=tk.X, padx=10, pady=5)
        
        ref_frame = self.ref_frame
        
        self.ref_path_var = tk.StringVar()
        ref_entry = ttk.Entry(ref_frame, textvariable=self.ref_path_var, width=60)
        ref_entry.pack(side=tk.LEFT, padx=5, fill=tk.X, expand=True)
        
        ref_browse_btn = ttk.Button(
            ref_frame,
            text="Browse",
            command=lambda: self.browse_pcd_file("reference")
        )
        ref_browse_btn.pack(side=tk.LEFT, padx=5)
        
        # Frame chọn Test PCD
        test_frame = ttk.LabelFrame(control_frame, text="Test PCD (Cần kiểm tra)", padding="10")
        test_frame.pack(fill=tk.X, padx=10, pady=5)
        
        self.test_path_var = tk.StringVar()
        test_entry = ttk.Entry(test_frame, textvariable=self.test_path_var, width=60)
        test_entry.pack(side=tk.LEFT, padx=5, fill=tk.X, expand=True)
        
        test_browse_btn = ttk.Button(
            test_frame,
            text="Browse",
            command=lambda: self.browse_pcd_file("test")
        )
        test_browse_btn.pack(side=tk.LEFT, padx=5)
        
        # Frame cấu hình
        config_frame = ttk.LabelFrame(control_frame, text="Cấu hình", padding="10")
        config_frame.pack(fill=tk.X, padx=10, pady=5)
        
        # Voxel size
        voxel_frame = ttk.Frame(config_frame)
        voxel_frame.pack(fill=tk.X, padx=5, pady=2)
        
        ttk.Label(voxel_frame, text="Voxel Size (m):", font=("Arial", 10)).pack(side=tk.LEFT, padx=5)
        self.voxel_size_var = tk.StringVar(value=str(DEFAULT_VOXEL_SIZE))
        voxel_entry = ttk.Entry(voxel_frame, textvariable=self.voxel_size_var, width=10)
        voxel_entry.pack(side=tk.LEFT, padx=5)
        
        # Remove outliers
        self.remove_outliers_var = tk.BooleanVar(value=True)
        outliers_check = ttk.Checkbutton(
            config_frame,
            text="Remove Outliers",
            variable=self.remove_outliers_var
        )
        outliers_check.pack(anchor=tk.W, padx=5, pady=2)
        
        # Use ICP
        self.use_icp_var = tk.BooleanVar(value=True)
        icp_check = ttk.Checkbutton(
            config_frame,
            text="Perform ICP Alignment",
            variable=self.use_icp_var
        )
        icp_check.pack(anchor=tk.W, padx=5, pady=2)
        
        # OBJ Conversion Settings (hidden by default)
        self.obj_settings_frame = ttk.LabelFrame(control_frame, text="OBJ Conversion Settings", padding="10")
        # Will be shown/hidden based on file type
        
        # Sampling method
        sampling_frame = ttk.Frame(self.obj_settings_frame)
        sampling_frame.pack(fill=tk.X, padx=5, pady=2)
        
        ttk.Label(sampling_frame, text="Sampling Method:", font=("Arial", 10)).pack(side=tk.LEFT, padx=5)
        self.obj_sampling_var = tk.StringVar(value="uniform")
        sampling_combo = ttk.Combobox(
            sampling_frame,
            textvariable=self.obj_sampling_var,
            values=["uniform", "poisson", "vertex"],
            state="readonly",
            width=15
        )
        sampling_combo.pack(side=tk.LEFT, padx=5)
        
        # Number of points
        points_frame = ttk.Frame(self.obj_settings_frame)
        points_frame.pack(fill=tk.X, padx=5, pady=2)
        
        ttk.Label(points_frame, text="Points:", font=("Arial", 10)).pack(side=tk.LEFT, padx=5)
        self.obj_points_var = tk.StringVar(value="")
        points_entry = ttk.Entry(points_frame, textvariable=self.obj_points_var, width=15)
        points_entry.pack(side=tk.LEFT, padx=5)
        ttk.Label(points_frame, text="(Auto if empty)", font=("Arial", 9), foreground="gray").pack(side=tk.LEFT, padx=5)
        
        # Match density
        self.match_density_var = tk.BooleanVar(value=False)
        match_check = ttk.Checkbutton(
            self.obj_settings_frame,
            text="Match PCD density",
            variable=self.match_density_var
        )
        match_check.pack(anchor=tk.W, padx=5, pady=2)
        
        # Save converted PCD
        self.save_obj_pcd_var = tk.BooleanVar(value=False)
        save_check = ttk.Checkbutton(
            self.obj_settings_frame,
            text="Save converted PCD",
            variable=self.save_obj_pcd_var,
            command=self.on_save_obj_pcd_changed
        )
        save_check.pack(anchor=tk.W, padx=5, pady=2)
        
        # Save path (hidden by default)
        self.save_path_frame = ttk.Frame(self.obj_settings_frame)
        self.save_path_var = tk.StringVar()
        save_path_entry = ttk.Entry(self.save_path_frame, textvariable=self.save_path_var, width=50)
        save_path_entry.pack(side=tk.LEFT, padx=5, fill=tk.X, expand=True)
        
        save_path_browse_btn = ttk.Button(
            self.save_path_frame,
            text="Browse",
            command=self.browse_save_path
        )
        save_path_browse_btn.pack(side=tk.LEFT, padx=5)
        
        # Frame nút điều khiển
        button_frame = ttk.Frame(control_frame)
        button_frame.pack(fill=tk.X, padx=10, pady=10)
        
        self.compare_btn = ttk.Button(
            button_frame,
            text="Compare PCD Files",
            command=self.start_comparison,
            style="Accent.TButton"
        )
        self.compare_btn.pack(side=tk.LEFT, padx=5)
        
        self.export_btn = ttk.Button(
            button_frame,
            text="Export JSON",
            command=self.export_results,
            state=tk.DISABLED
        )
        self.export_btn.pack(side=tk.LEFT, padx=5)
        
        # Progress bar
        self.progress_var = tk.StringVar(value="Ready")
        self.progress_label = ttk.Label(
            button_frame,
            textvariable=self.progress_var,
            foreground="gray"
        )
        self.progress_label.pack(side=tk.LEFT, padx=20)
        
        self.progress_bar = ttk.Progressbar(
            button_frame,
            mode='indeterminate',
            length=200
        )
        self.progress_bar.pack(side=tk.LEFT, padx=5)
        
        # Frame kết quả
        results_frame = ttk.LabelFrame(self, text="Kết quả so sánh", padding="10")
        results_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Results display
        self.results_text = scrolledtext.ScrolledText(
            results_frame,
            height=20,
            wrap=tk.WORD,
            state=tk.DISABLED,
            font=("Consolas", 10)
        )
        self.results_text.pack(fill=tk.BOTH, expand=True)
        
        # Log area
        log_frame = ttk.LabelFrame(self, text="Log", padding="5")
        log_frame.pack(fill=tk.X, padx=10, pady=5)
        
        self.log_text = scrolledtext.ScrolledText(
            log_frame,
            height=8,
            wrap=tk.WORD,
            state=tk.DISABLED
        )
        self.log_text.pack(fill=tk.BOTH, expand=True)
    
    def log(self, message):
        """Thêm message vào log"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.log_text.config(state=tk.NORMAL)
        self.log_text.insert(tk.END, f"[{timestamp}] {message}\n")
        self.log_text.see(tk.END)
        self.log_text.config(state=tk.DISABLED)
    
    def _get_voxel_size(self) -> float:
        """Lấy và validate voxel size từ UI"""
        try:
            voxel_size = float(self.voxel_size_var.get())
            if voxel_size < 0:
                voxel_size = DEFAULT_VOXEL_SIZE
                self.log(f"⚠️  Voxel size không hợp lệ, sử dụng mặc định {DEFAULT_VOXEL_SIZE}")
            return voxel_size
        except ValueError:
            self.log(f"⚠️  Voxel size không hợp lệ, sử dụng mặc định {DEFAULT_VOXEL_SIZE}")
            return DEFAULT_VOXEL_SIZE
    
    def _validate_paths(self) -> Optional[Tuple[Path, Path]]:
        """Validate và trả về paths, hoặc None nếu có lỗi"""
        ref_path = self.ref_path_var.get()
        test_path = self.test_path_var.get()
        
        if not ref_path:
            ref_label = "OBJ" if self.file_type == "obj" else "PCD"
            messagebox.showerror("Lỗi", f"Vui lòng chọn Reference {ref_label} file")
            return None
        
        if not test_path:
            messagebox.showerror("Lỗi", "Vui lòng chọn Test PCD file")
            return None
        
        ref_path_obj = Path(ref_path)
        test_path_obj = Path(test_path)
        
        if not ref_path_obj.exists():
            ref_label = "OBJ" if self.file_type == "obj" else "PCD"
            messagebox.showerror("Lỗi", f"Reference {ref_label} file không tồn tại: {ref_path}")
            return None
        
        if not test_path_obj.exists():
            messagebox.showerror("Lỗi", f"Test PCD file không tồn tại: {test_path}")
            return None
        
        return ref_path_obj, test_path_obj
    
    def _get_obj_settings(self, ref_path: Path) -> Tuple[str, Optional[int], bool, Optional[Path]]:
        """Lấy OBJ conversion settings từ UI"""
        obj_sampling = self.obj_sampling_var.get() if self.file_type == "obj" else "uniform"
        obj_points = None
        
        if self.file_type == "obj":
            try:
                points_str = self.obj_points_var.get().strip()
                if points_str:
                    obj_points = int(points_str)
            except ValueError:
                pass
        
        match_density = self.match_density_var.get() if self.file_type == "obj" else False
        
        save_obj_pcd_path = None
        if self.file_type == "obj" and self.save_obj_pcd_var.get():
            save_path_str = self.save_path_var.get().strip()
            if save_path_str:
                save_obj_pcd_path = Path(save_path_str)
            else:
                # Auto-generate
                save_obj_pcd_path = ref_path.parent / f"{ref_path.stem}_converted.pcd"
        
        return obj_sampling, obj_points, match_density, save_obj_pcd_path
    
    def on_file_type_changed(self):
        """Handle file type change"""
        self.file_type = self.file_type_var.get()
        
        if self.file_type == "obj":
            # Update reference frame label
            self.ref_frame.config(text="Reference OBJ (Thiết kế chuẩn)")
            # Show OBJ settings
            self.obj_settings_frame.pack(fill=tk.X, padx=10, pady=5)
        else:
            # Update reference frame label
            self.ref_frame.config(text="Reference PCD (Chuẩn)")
            # Hide OBJ settings
            self.obj_settings_frame.pack_forget()
            # Hide save path
            self.save_path_frame.pack_forget()
            self.save_obj_pcd_var.set(False)
    
    def on_save_obj_pcd_changed(self):
        """Handle save OBJ PCD checkbox change"""
        if self.save_obj_pcd_var.get():
            self.save_path_frame.pack(fill=tk.X, padx=5, pady=2)
            # Set default path if empty
            if not self.save_path_var.get():
                ref_path = self.ref_path_var.get()
                if ref_path:
                    ref_path_obj = Path(ref_path)
                    default_path = ref_path_obj.parent / f"{ref_path_obj.stem}_converted.pcd"
                    self.save_path_var.set(str(default_path))
        else:
            self.save_path_frame.pack_forget()
    
    def browse_save_path(self):
        """Browse for save path"""
        current_path = self.save_path_var.get()
        if not current_path:
            ref_path = self.ref_path_var.get()
            if ref_path:
                current_path = str(Path(ref_path).parent)
            else:
                current_path = str(self.default_pcd_dir) if self.default_pcd_dir.exists() else "."
        
        file = filedialog.asksaveasfilename(
            title="Lưu converted PCD",
            initialdir=current_path,
            defaultextension=".pcd",
            filetypes=[("PCD files", "*.pcd"), ("All files", "*.*")]
        )
        
        if file:
            self.save_path_var.set(file)
    
    def browse_pcd_file(self, file_type):
        """Browse cho PCD hoặc OBJ file"""
        current_path = None
        if file_type == "reference":
            current_path = self.ref_path_var.get()
        else:
            current_path = self.test_path_var.get()
        
        if not current_path:
            current_path = str(self.default_pcd_dir) if self.default_pcd_dir.exists() else ""
        
        # Determine file types based on file_type and mode
        if file_type == "reference" and self.file_type == "obj":
            filetypes = [("OBJ files", "*.obj"), ("All files", "*.*")]
            title = f"Chọn {file_type.capitalize()} OBJ file"
        else:
            filetypes = [("PCD files", "*.pcd"), ("All files", "*.*")]
            title = f"Chọn {file_type.capitalize()} PCD file"
        
        file = filedialog.askopenfilename(
            title=title,
            initialdir=current_path or ".",
            filetypes=filetypes
        )
        
        if file:
            if file_type == "reference":
                self.ref_path_var.set(file)
                self.reference_pcd_path = Path(file)
                # Update default save path if OBJ
                if self.file_type == "obj" and self.save_obj_pcd_var.get():
                    ref_path_obj = Path(file)
                    default_path = ref_path_obj.parent / f"{ref_path_obj.stem}_converted.pcd"
                    self.save_path_var.set(str(default_path))
            else:
                self.test_path_var.set(file)
                self.test_pcd_path = Path(file)
            
            file_label = "OBJ" if (file_type == "reference" and self.file_type == "obj") else "PCD"
            self.log(f"Đã chọn {file_type} {file_label}: {Path(file).name}")
    
    def start_comparison(self):
        """Bắt đầu so sánh PCD"""
        if self.is_comparing:
            messagebox.showwarning("Cảnh báo", "Đang thực hiện so sánh, vui lòng đợi...")
            return
        
        # Validate paths
        paths = self._validate_paths()
        if paths is None:
            return
        
        ref_path_obj, test_path_obj = paths
        
        # Get configuration
        voxel_size = self._get_voxel_size()
        remove_outliers = self.remove_outliers_var.get()
        use_icp = self.use_icp_var.get()
        
        # Get OBJ settings if applicable
        obj_sampling, obj_points, match_density, save_obj_pcd_path = self._get_obj_settings(ref_path_obj)
        
        # Start comparison in thread
        self._start_comparison_thread(
            ref_path_obj, test_path_obj, voxel_size, remove_outliers, use_icp,
            self.file_type, obj_sampling, obj_points, match_density, save_obj_pcd_path
        )
    
    def _start_comparison_thread(self, ref_path, test_path, voxel_size, remove_outliers, use_icp,
                                 file_type, obj_sampling, obj_points, match_density, save_obj_pcd_path):
        """Khởi động comparison thread"""
        self.is_comparing = True
        self.compare_btn.config(state=tk.DISABLED)
        self.progress_bar.start()
        self.progress_var.set("Đang so sánh...")
        self.results_text.config(state=tk.NORMAL)
        self.results_text.delete('1.0', tk.END)
        self.results_text.insert(tk.END, "Đang thực hiện so sánh...\n")
        self.results_text.config(state=tk.DISABLED)
        
        self.log(f"Bắt đầu so sánh: {ref_path.name} vs {test_path.name}")
        
        self.comparison_thread = threading.Thread(
            target=self.run_comparison,
            args=(ref_path, test_path, voxel_size, remove_outliers, use_icp,
                  file_type, obj_sampling, obj_points, match_density, save_obj_pcd_path),
            daemon=True
        )
        self.comparison_thread.start()
    
    def run_comparison(self, ref_path, test_path, voxel_size, remove_outliers, use_icp,
                      file_type, obj_sampling, obj_points, match_density, save_obj_pcd_path):
        """Chạy comparison trong thread"""
        try:
            # Load files
            self.after(0, lambda: self.progress_var.set("Đang load files..."))
            self.after(0, lambda: self.log("Loading files..."))
            
            # Load reference (PCD or OBJ)
            if file_type == "obj":
                self.after(0, lambda: self.log(f"Converting OBJ to point cloud (method: {obj_sampling})..."))
                
                # Load test PCD first if matching density
                match_density_count = None
                if match_density:
                    pcd_test_temp = load_pcd(test_path)
                    match_density_count = len(pcd_test_temp.points)
                    self.after(0, lambda: self.log(f"Matching PCD density: {match_density_count:,} points"))
                
                pcd_ref = load_obj_as_pointcloud(
                    ref_path,
                    sampling_method=obj_sampling,
                    num_points=obj_points,
                    match_density=match_density_count
                )
                self.after(0, lambda: self.log(f"Converted OBJ: {len(pcd_ref.points):,} points"))
            else:
                pcd_ref = load_pcd(ref_path)
            
            # Load test PCD
            pcd_test = load_pcd(test_path)
            
            ref_points = len(pcd_ref.points)
            test_points = len(pcd_test.points)
            
            self.after(0, lambda: self.log(f"Reference: {ref_points:,} points"))
            self.after(0, lambda: self.log(f"Test: {test_points:,} points"))
            
            # Preprocess
            self.after(0, lambda: self.progress_var.set("Đang preprocessing..."))
            self.after(0, lambda: self.log("Preprocessing point clouds..."))
            
            pcd_ref = preprocess_pcd(
                pcd_ref,
                voxel_size=voxel_size,
                remove_outliers=remove_outliers
            )
            
            pcd_test = preprocess_pcd(
                pcd_test,
                voxel_size=voxel_size,
                remove_outliers=remove_outliers
            )
            
            self.after(0, lambda: self.log(f"After preprocessing - Reference: {len(pcd_ref.points):,} points"))
            self.after(0, lambda: self.log(f"After preprocessing - Test: {len(pcd_test.points):,} points"))
            
            # Compute metrics
            self.after(0, lambda: self.progress_var.set("Đang tính toán metrics..."))
            self.after(0, lambda: self.log("Computing similarity metrics..."))
            
            # Auto-adjust ICP threshold based on voxel_size
            icp_threshold = voxel_size * DEFAULT_ICP_THRESHOLD_MULTIPLIER
            self.after(0, lambda: self.log(f"ICP threshold (auto): {icp_threshold:.3f} m"))
            
            # Save converted PCD if requested
            save_pcd_path = save_obj_pcd_path if file_type == "obj" else None
            if save_pcd_path:
                self.after(0, lambda: self.log(f"Saving converted PCD to: {save_pcd_path}"))
            
            metrics = compute_similarity_metrics(
                pcd_ref,
                pcd_test,
                perform_icp=use_icp,
                icp_threshold=icp_threshold,
                icp_max_iterations=DEFAULT_ICP_MAX_ITERATIONS,
                save_obj_pcd=save_pcd_path
            )
            
            if save_pcd_path:
                self.after(0, lambda: self.log(f"✅ Converted PCD saved to: {save_pcd_path}"))
            
            # Calculate similarity
            similarity = calculate_similarity_percentage(metrics)
            
            # Detect drift
            drift_detected, drift_reasons = detect_drift(metrics, similarity)
            
            # Store results
            self.last_results = {
                "reference_file": str(ref_path),
                "test_file": str(test_path),
                "reference_points": metrics["reference_points"],
                "test_points": metrics["test_points"],
                "metrics": metrics,
                "similarity": similarity,
                "drift_detected": drift_detected,
                "drift_reasons": drift_reasons
            }
            
            # Display results
            self.after(0, lambda: self.display_results())
            
        except Exception as e:
            error_msg = f"Lỗi khi so sánh: {e}"
            self.after(0, lambda: self.log(f"❌ {error_msg}"))
            self.after(0, lambda: messagebox.showerror("Lỗi", error_msg))
            import traceback
            self.after(0, lambda: self.log(traceback.format_exc()))
        finally:
            self.is_comparing = False
            self.after(0, lambda: self.progress_bar.stop())
            self.after(0, lambda: self.progress_var.set("Hoàn thành"))
            self.after(0, lambda: self.compare_btn.config(state=tk.NORMAL))
    
    def display_results(self):
        """Hiển thị kết quả"""
        if not self.last_results:
            return
        
        results = self.last_results
        metrics = results["metrics"]
        
        self.results_text.config(state=tk.NORMAL)
        self.results_text.delete('1.0', tk.END)
        
        # Format results
        output = []
        output.append("=" * 60)
        output.append("PCD COMPARISON RESULTS")
        output.append("=" * 60)
        output.append("")
        output.append(f"Reference PCD: {Path(results['reference_file']).name}")
        output.append(f"Test PCD: {Path(results['test_file']).name}")
        output.append("")
        output.append("Point Cloud Info:")
        output.append(f"  Reference: {results['reference_points']:,} points")
        output.append(f"  Test: {results['test_points']:,} points")
        output.append("")
        output.append("Metrics:")
        output.append(f"  ICP Fitness Score: {metrics['icp_fitness']:.4f}")
        output.append(f"  ICP Inlier RMSE: {metrics['icp_inlier_rmse']:.6f} m")
        output.append(f"  Hausdorff Distance: {metrics['hausdorff_distance']:.6f} m")
        output.append(f"  Chamfer Distance: {metrics['chamfer_distance']:.6f} m")
        output.append(f"  Bounding Box Overlap: {metrics['bbox_overlap']:.4f}")
        output.append(f"  Centroid Distance: {metrics['centroid_distance']:.6f} m")
        output.append(f"  Point Density Ratio: {metrics['density_ratio']:.4f}")
        output.append("")
        output.append(f"Similarity: {results['similarity']:.2f}%")
        output.append(f"Drift Detected: {'Yes' if results['drift_detected'] else 'No'}")
        
        if results['drift_detected'] and results['drift_reasons']:
            output.append("")
            output.append("Drift Reasons:")
            for reason in results['drift_reasons']:
                output.append(f"  - {reason}")
        
        output.append("")
        output.append("=" * 60)
        
        self.results_text.insert(tk.END, "\n".join(output))
        self.results_text.config(state=tk.DISABLED)
        
        # Enable export button
        self.export_btn.config(state=tk.NORMAL)
        
        # Log completion
        status = "❌ DRIFT DETECTED" if results['drift_detected'] else "✅ No drift detected"
        self.log(f"Hoàn thành: Similarity = {results['similarity']:.2f}% - {status}")
    
    def export_results(self):
        """Export kết quả ra file JSON"""
        if not self.last_results:
            messagebox.showwarning("Cảnh báo", "Chưa có kết quả để export")
            return
        
        # Suggest filename
        if self.test_pcd_path:
            default_filename = f"{self.test_pcd_path.stem}_comparison.json"
            default_dir = str(self.test_pcd_path.parent)
        else:
            default_filename = "pcd_comparison.json"
            default_dir = str(self.default_pcd_dir) if self.default_pcd_dir.exists() else "."
        
        file = filedialog.asksaveasfilename(
            title="Lưu kết quả JSON",
            initialdir=default_dir,
            initialfile=default_filename,
            defaultextension=".json",
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")]
        )
        
        if file:
            try:
                # Prepare JSON data
                json_data = {
                    "timestamp": datetime.now().isoformat(),
                    "reference_file": self.last_results["reference_file"],
                    "test_file": self.last_results["test_file"],
                    "reference_points": self.last_results["reference_points"],
                    "test_points": self.last_results["test_points"],
                    "metrics": {
                        "icp_fitness": self.last_results["metrics"]["icp_fitness"],
                        "icp_inlier_rmse": self.last_results["metrics"]["icp_inlier_rmse"],
                        "hausdorff_distance": self.last_results["metrics"]["hausdorff_distance"],
                        "chamfer_distance": self.last_results["metrics"]["chamfer_distance"],
                        "bbox_overlap": self.last_results["metrics"]["bbox_overlap"],
                        "centroid_distance": self.last_results["metrics"]["centroid_distance"],
                        "density_ratio": self.last_results["metrics"]["density_ratio"]
                    },
                    "similarity_percentage": self.last_results["similarity"],
                    "drift_detected": self.last_results["drift_detected"],
                    "drift_reasons": self.last_results["drift_reasons"]
                }
                
                with open(file, 'w', encoding='utf-8') as f:
                    json.dump(json_data, f, indent=2, ensure_ascii=False)
                
                self.log(f"✅ Đã export kết quả: {Path(file).name}")
                messagebox.showinfo("Thành công", f"Đã lưu kết quả vào:\n{file}")
                
            except Exception as e:
                error_msg = f"Lỗi khi export: {e}"
                self.log(f"❌ {error_msg}")
                messagebox.showerror("Lỗi", error_msg)
