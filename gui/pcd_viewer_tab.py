#!/usr/bin/env python3
"""
PCD Viewer Tab Module
Chứa PCDViewerTab để hiển thị PCD files trong RViz
"""

import threading
import subprocess
import queue
from pathlib import Path
from datetime import datetime
import os
import tempfile
import shutil
from functools import partial

try:
    import psutil
    HAS_PSUTIL = True
except ImportError:
    HAS_PSUTIL = False

try:
    import open3d as o3d
    HAS_OPEN3D = True
except ImportError:
    HAS_OPEN3D = False

try:
    import tkinter as tk
    from tkinter import ttk, messagebox, scrolledtext, filedialog
except ImportError as e:
    print(f"Lỗi import: {e}")
    import sys
    sys.exit(1)


class PCDViewerTab(ttk.Frame):
    """Tab cho PCD Viewer"""
    
    def __init__(self, parent):
        super().__init__(parent)
        
        # Paths
        self.workspace_path = Path(__file__).parent.parent / "ws"
        self.fast_livo_path = self.workspace_path / "src" / "FAST-LIVO2"
        self.default_pcd_dir = self.fast_livo_path / "Log" / "PCD"
        
        # Processes
        self.pcd_viewer_process = None
        self.rviz_process = None
        
        # State
        self.is_viewer_running = False
        self.is_rviz_running = False
        self.pcd_file_path = None
        
        # File size limits (MB) - để tránh crash với file quá lớn
        # Giảm giới hạn vì file 400MB đã gây crash
        self.max_file_size_mb = 300  # 300MB - giới hạn an toàn
        self.warning_file_size_mb = 200  # 200MB - cảnh báo
        self.safe_file_size_mb = 100  # 100MB - kích thước an toàn
        
        # Tạo UI
        self.create_widgets()
    
    def create_widgets(self):
        """Tạo các widget cho tab PCD Viewer"""
        
        # Title
        title_label = ttk.Label(
            self,
            text="PCD Map Viewer",
            font=("Arial", 16, "bold")
        )
        title_label.pack(pady=10)
        
        # Frame điều khiển
        control_frame = ttk.Frame(self, padding="10")
        control_frame.pack(fill=tk.X)
        
        # Frame chọn nguồn PCD
        source_frame = ttk.LabelFrame(control_frame, text="Nguồn PCD File", padding="10")
        source_frame.pack(fill=tk.X, padx=10, pady=5)
        
        self.pcd_source_var = tk.StringVar(value="default")  # "default" hoặc "anhsong"
        
        source_default_radio = ttk.Radiobutton(
            source_frame,
            text="Đường dẫn mặc định (Log/PCD)",
            variable=self.pcd_source_var,
            value="default",
            command=self.on_source_changed
        )
        source_default_radio.pack(side=tk.LEFT, padx=10)
        
        source_anhsong_radio = ttk.Radiobutton(
            source_frame,
            text="External Drive (/media/an/ANHSON)",
            variable=self.pcd_source_var,
            value="anhsong",
            command=self.on_source_changed
        )
        source_anhsong_radio.pack(side=tk.LEFT, padx=10)
        
        # Frame chọn PCD file
        pcd_frame = ttk.LabelFrame(control_frame, text="PCD File", padding="10")
        pcd_frame.pack(fill=tk.X, padx=10, pady=5)
        
        self.pcd_path_var = tk.StringVar()
        pcd_entry = ttk.Entry(pcd_frame, textvariable=self.pcd_path_var, width=60, state=tk.DISABLED)
        pcd_entry.pack(side=tk.LEFT, padx=5, fill=tk.X, expand=True)
        
        # Nút Browse để chọn file
        browse_btn = ttk.Button(
            pcd_frame,
            text="Chọn File",
            command=self.browse_pcd_file
        )
        browse_btn.pack(side=tk.LEFT, padx=5)
        
        # Ẩn các nút Browse và Auto Find - không cho người dùng chọn file
        # browse_btn = ttk.Button(
        #     pcd_frame,
        #     text="Browse",
        #     command=self.browse_pcd_file
        # )
        # browse_btn.pack(side=tk.LEFT, padx=5)
        
        # auto_find_btn = ttk.Button(
        #     pcd_frame,
        #     text="Auto Find",
        #     command=self.auto_find_pcd
        # )
        # auto_find_btn.pack(side=tk.LEFT, padx=5)
        
        # Frame cấu hình
        config_frame = ttk.LabelFrame(control_frame, text="Cấu hình", padding="10")
        config_frame.pack(fill=tk.X, padx=10, pady=5)
        
        # Topic name
        topic_frame = ttk.Frame(config_frame)
        topic_frame.pack(fill=tk.X, padx=5, pady=2)
        
        ttk.Label(
            topic_frame,
            text="Topic Name:",
            font=("Arial", 10)
        ).pack(side=tk.LEFT, padx=5)
        
        self.topic_var = tk.StringVar(value="/pcd_map")
        topic_entry = ttk.Entry(
            topic_frame,
            textvariable=self.topic_var,
            width=20
        )
        topic_entry.pack(side=tk.LEFT, padx=5)
        
        # Frame ID
        frame_id_frame = ttk.Frame(config_frame)
        frame_id_frame.pack(fill=tk.X, padx=5, pady=2)
        
        ttk.Label(
            frame_id_frame,
            text="Frame ID:",
            font=("Arial", 10)
        ).pack(side=tk.LEFT, padx=5)
        
        self.frame_id_var = tk.StringVar(value="map")
        frame_id_entry = ttk.Entry(
            frame_id_frame,
            textvariable=self.frame_id_var,
            width=20
        )
        frame_id_entry.pack(side=tk.LEFT, padx=5)
        
        # Publish rate
        rate_frame = ttk.Frame(config_frame)
        rate_frame.pack(fill=tk.X, padx=5, pady=2)
        
        ttk.Label(
            rate_frame,
            text="Publish Rate (Hz):",
            font=("Arial", 10)
        ).pack(side=tk.LEFT, padx=5)
        
        self.rate_var = tk.StringVar(value="2.0")  # Increased from 1.0 to 2.0 Hz for smoother display
        rate_spinbox = ttk.Spinbox(
            rate_frame,
            from_=0.0,
            to=10.0,
            increment=0.1,
            textvariable=self.rate_var,
            width=10,
            format="%.1f"
        )
        rate_spinbox.pack(side=tk.LEFT, padx=5)
        
        ttk.Label(
            rate_frame,
            text="(0.0 = publish once)",
            font=("Arial", 9),
            foreground="gray"
        ).pack(side=tk.LEFT, padx=10)
        
        # Loop option
        self.loop_var = tk.BooleanVar(value=True)
        loop_checkbox = ttk.Checkbutton(
            config_frame,
            text="Loop (publish liên tục)",
            variable=self.loop_var
        )
        loop_checkbox.pack(anchor=tk.W, padx=5, pady=2)
        
        # Use RViz option
        self.use_rviz_var = tk.BooleanVar(value=True)
        rviz_checkbox = ttk.Checkbutton(
            config_frame,
            text="Launch RViz",
            variable=self.use_rviz_var
        )
        rviz_checkbox.pack(anchor=tk.W, padx=5, pady=2)
        
        # Skip large files option
        self.skip_large_files_var = tk.BooleanVar(value=True)
        skip_large_checkbox = ttk.Checkbutton(
            config_frame,
            text=f"Bỏ qua file quá lớn (> {self.max_file_size_mb} MB) khi tự động tìm",
            variable=self.skip_large_files_var
        )
        skip_large_checkbox.pack(anchor=tk.W, padx=5, pady=2)
        
        # Auto downsample large files option
        self.auto_downsample_var = tk.BooleanVar(value=True)
        downsample_checkbox = ttk.Checkbutton(
            config_frame,
            text=f"Tự động downsample file > {self.max_file_size_mb} MB trước khi load",
            variable=self.auto_downsample_var
        )
        downsample_checkbox.pack(anchor=tk.W, padx=5, pady=2)
        
        # Voxel size for downsampling
        voxel_frame = ttk.Frame(config_frame)
        voxel_frame.pack(fill=tk.X, padx=5, pady=2)
        
        ttk.Label(
            voxel_frame,
            text="Voxel size (m) cho downsample:",
            font=("Arial", 10)
        ).pack(side=tk.LEFT, padx=5)
        
        self.voxel_size_var = tk.StringVar(value="0.15")
        voxel_entry = ttk.Entry(
            voxel_frame,
            textvariable=self.voxel_size_var,
            width=10
        )
        voxel_entry.pack(side=tk.LEFT, padx=5)
        
        ttk.Label(
            voxel_frame,
            text="(lớn hơn = ít điểm hơn, nhanh hơn)",
            font=("Arial", 9),
            foreground="gray"
        ).pack(side=tk.LEFT, padx=10)
        
        # Frame nút điều khiển
        button_frame = ttk.Frame(control_frame)
        button_frame.pack(fill=tk.X, padx=10, pady=10)
        
        self.start_btn = ttk.Button(
            button_frame,
            text="Start PCD Viewer",
            command=self.start_viewer,
            style="Accent.TButton"
        )
        self.start_btn.pack(side=tk.LEFT, padx=5)
        
        self.stop_btn = ttk.Button(
            button_frame,
            text="Stop PCD Viewer",
            command=self.stop_viewer,
            state=tk.DISABLED
        )
        self.stop_btn.pack(side=tk.LEFT, padx=5)
        
        # Label trạng thái
        self.status_label = ttk.Label(
            button_frame,
            text="Trạng thái: Sẵn sàng",
            foreground="gray"
        )
        self.status_label.pack(side=tk.LEFT, padx=20)
        
        # Frame thông tin PCD
        info_frame = ttk.LabelFrame(self, text="Thông tin PCD", padding="10")
        info_frame.pack(fill=tk.X, padx=10, pady=5)
        
        self.info_label = ttk.Label(
            info_frame,
            text="Chưa chọn PCD file",
            font=("Arial", 10)
        )
        self.info_label.pack(anchor=tk.W, padx=5)
        
        # Text area để hiển thị log
        log_frame = ttk.LabelFrame(self, text="Log", padding="5")
        log_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        self.log_text = scrolledtext.ScrolledText(
            log_frame,
            height=15,
            wrap=tk.WORD,
            state=tk.DISABLED
        )
        self.log_text.pack(fill=tk.BOTH, expand=True)
        
        # Tự động tìm và chọn file PCD có màu sắc khi khởi động (trong thread riêng với delay để tránh crash)
        # Chỉ tìm trong nguồn default, không tìm trong ANHSON để tránh crash
        # Delay 1 giây để UI hoàn toàn load xong trước khi tìm file
        def delayed_auto_find():
            import time
            time.sleep(1.0)  # Delay 1 giây
            self.auto_find_rgb_pcd_safe()
        threading.Thread(target=delayed_auto_find, daemon=True).start()
    
    def on_source_changed(self):
        """Xử lý khi người dùng thay đổi nguồn PCD"""
        source = self.pcd_source_var.get()
        if source == "anhsong":
            self.log("🔄 Đã chuyển sang nguồn: External Drive (/media/an/ANHSON)")
            # Tự động mở file dialog để chọn file (delay lớn hơn để tránh crash)
            self.after(500, self.browse_pcd_file_anhsong)  # Delay 500ms để UI update hoàn toàn
        else:
            self.log("🔄 Đã chuyển sang nguồn: Đường dẫn mặc định")
            # Tự động tìm lại file trong nguồn mới (trong thread riêng)
            threading.Thread(target=self.auto_find_rgb_pcd_safe, daemon=True).start()
    
    def auto_find_rgb_pcd_safe(self):
        """Wrapper an toàn cho auto_find_rgb_pcd để tránh crash"""
        try:
            self.auto_find_rgb_pcd()
        except Exception as e:
            self.log(f"❌ Lỗi khi tự động tìm file PCD: {e}")
            self.info_label.config(text="Lỗi khi tìm file PCD")
    
    def log(self, message):
        """Thêm message vào log"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.log_text.config(state=tk.NORMAL)
        self.log_text.insert(tk.END, f"[{timestamp}] {message}\n")
        self.log_text.see(tk.END)
        self.log_text.config(state=tk.DISABLED)
    
    def get_file_size_mb(self, file_path):
        """Lấy kích thước file theo MB - an toàn với try-catch"""
        try:
            file_path_obj = Path(file_path)
            if not file_path_obj.exists():
                return 0
            
            # Sử dụng stat với timeout để tránh hang
            try:
                stat_info = file_path_obj.stat()
                size_mb = stat_info.st_size / (1024 * 1024)
                # Giới hạn size tối đa để tránh overflow
                if size_mb > 10000:  # > 10GB - có thể là lỗi
                    return 0
                return size_mb
            except (OSError, IOError, PermissionError, MemoryError):
                return 0
        except (OSError, IOError, PermissionError, MemoryError):
            # Lỗi khi truy cập file - return 0 để skip file này
            return 0
        except Exception:
            # Bất kỳ lỗi nào khác - return 0 để tránh crash
            return 0
    
    def check_memory_available(self):
        """Kiểm tra memory available (GB)"""
        if not HAS_PSUTIL:
            return None  # Không thể kiểm tra
        try:
            memory = psutil.virtual_memory()
            return memory.available / (1024 ** 3)  # GB
        except Exception:
            return None
    
    def is_file_too_large(self, file_path, skip_if_large=True):
        """
        Kiểm tra xem file có quá lớn không
        skip_if_large: Nếu True, sẽ skip file nếu quá lớn (dùng cho auto_find)
        """
        file_size_mb = self.get_file_size_mb(file_path)
        
        if file_size_mb > self.max_file_size_mb:
            if skip_if_large:
                self.log(f"⚠️  Bỏ qua file quá lớn: {Path(file_path).name} ({file_size_mb:.2f} MB > {self.max_file_size_mb} MB)")
            return True, file_size_mb
        
        if file_size_mb > self.warning_file_size_mb:
            self.log(f"⚠️  Cảnh báo: File lớn {Path(file_path).name} ({file_size_mb:.2f} MB)")
            return False, file_size_mb
        
        return False, file_size_mb
    
    def browse_pcd_file(self):
        """Browse cho PCD file"""
        try:
            # Xác định thư mục ban đầu
            source = self.pcd_source_var.get() if hasattr(self, 'pcd_source_var') else "default"
            
            if source == "anhsong":
                initial_dir = "/media/an/ANHSON"
            else:
                initial_dir = self.pcd_path_var.get() or str(self.default_pcd_dir)
            
            file = filedialog.askopenfilename(
                title="Chọn PCD file",
                initialdir=initial_dir,
                filetypes=[("PCD files", "*.pcd"), ("All files", "*.*")]
            )
            if file:
                self.pcd_path_var.set(file)
                # Chạy update info trong thread riêng để tránh crash
                threading.Thread(target=self._update_pcd_info_safe, args=(file,), daemon=True).start()
                self.log(f"✅ Đã chọn PCD file: {Path(file).name}")
                self.log(f"📁 Đường dẫn: {file}")
        except Exception as e:
            self.log(f"❌ Lỗi khi chọn file: {e}")
            messagebox.showerror("Lỗi", f"Không thể chọn file: {e}")
    
    def browse_pcd_file_anhsong(self):
        """Mở file dialog để chọn file PCD từ ANHSON"""
        try:
            external_drive_path = Path("/media/an/ANHSON")
            
            if not external_drive_path.exists():
                messagebox.showerror(
                    "Lỗi",
                    f"Thư mục external drive không tồn tại: {external_drive_path}\n\n"
                    "Vui lòng kiểm tra lại đường dẫn."
                )
                return
            
            file = filedialog.askopenfilename(
                title="Chọn PCD file từ ANHSON",
                initialdir=str(external_drive_path),
                filetypes=[("PCD files", "*.pcd"), ("All files", "*.*")]
            )
            
            if file:
                self.pcd_path_var.set(file)
                # Chạy update info trong thread riêng để tránh crash
                threading.Thread(target=self._update_pcd_info_safe, args=(file,), daemon=True).start()
                self.log(f"✅ Đã chọn PCD file từ ANHSON: {Path(file).name}")
                self.log(f"📁 Đường dẫn: {file}")
        except Exception as e:
            self.log(f"❌ Lỗi khi chọn file từ ANHSON: {e}")
            messagebox.showerror("Lỗi", f"Không thể chọn file: {e}")
    
    def downsample_pcd_file(self, input_path, voxel_size=0.15):
        """
        Downsample PCD file sử dụng open3d
        Returns: path to downsampled file hoặc None nếu lỗi
        """
        if not HAS_OPEN3D:
            self.log("❌ open3d không được cài đặt. Không thể downsample file.")
            return None
        
        try:
            self.log(f"📉 Đang downsample file: {Path(input_path).name} (voxel_size={voxel_size}m)...")
            
            # Đọc file PCD
            pcd = o3d.io.read_point_cloud(str(input_path))
            if len(pcd.points) == 0:
                self.log("❌ File PCD rỗng hoặc không đọc được")
                return None
            
            original_points = len(pcd.points)
            self.log(f"   Điểm gốc: {original_points:,} points")
            
            # Downsample
            downsampled_pcd = pcd.voxel_down_sample(voxel_size=voxel_size)
            downsampled_points = len(downsampled_pcd.points)
            self.log(f"   Điểm sau downsample: {downsampled_points:,} points ({downsampled_points/original_points*100:.1f}%)")
            
            # Tạo temp file để lưu file đã downsample
            temp_dir = Path(tempfile.gettempdir()) / "pcd_viewer"
            temp_dir.mkdir(parents=True, exist_ok=True)
            
            # Tên file dựa trên file gốc
            original_name = Path(input_path).stem
            output_path = temp_dir / f"{original_name}_downsampled_{voxel_size}.pcd"
            
            # Lưu file đã downsample
            success = o3d.io.write_point_cloud(str(output_path), downsampled_pcd, write_ascii=False)
            if not success:
                self.log("❌ Không thể lưu file đã downsample")
                return None
            
            output_size_mb = output_path.stat().st_size / (1024 * 1024)
            self.log(f"✅ Đã downsample và lưu: {output_path.name} ({output_size_mb:.2f} MB)")
            
            return str(output_path)
            
        except Exception as e:
            self.log(f"❌ Lỗi khi downsample file: {e}")
            return None
    
    def check_pcd_has_rgb(self, pcd_path):
        """Kiểm tra xem file PCD có chứa thông tin màu sắc RGB hay không"""
        try:
            # Kiểm tra file size trước - nếu quá lớn, không check để tránh crash
            file_size_mb = self.get_file_size_mb(pcd_path)
            # Giảm ngưỡng xuống 100MB để tránh crash hoàn toàn
            if file_size_mb > 100:  # > 100MB - không check để tránh crash
                return False
            
            # Đọc với buffer nhỏ để tránh load toàn bộ file
            buffering = 4096 if file_size_mb > 20 else None
            with open(pcd_path, 'rb', buffering=buffering) as f:
                # Đọc header của file PCD - giới hạn số bytes đọc
                header_lines = []
                bytes_read = 0
                max_header_bytes = 10240  # Chỉ đọc tối đa 10KB header
                
                for _ in range(30):  # Đọc tối đa 30 dòng đầu
                    if bytes_read >= max_header_bytes:
                        break
                    line_bytes = f.readline()
                    bytes_read += len(line_bytes)
                    if not line_bytes:
                        break
                    try:
                        line = line_bytes.decode('utf-8', errors='ignore').strip()
                        if not line or line.startswith('DATA'):
                            break
                        header_lines.append(line)
                    except Exception:
                        break  # Nếu không decode được, dừng lại
            
            # Tìm dòng FIELDS
            try:
                fields_line = None
                for line in header_lines:
                    if line.startswith('FIELDS'):
                        fields_line = line
                        break
                
                fields = None
                if fields_line:
                    fields = fields_line.split()[1:]  # Bỏ qua từ "FIELDS"
                    # Kiểm tra xem có r, g, b hoặc rgb trong fields
                    # PointXYZRGB có fields: x y z rgb
                    # Hoặc có thể có: x y z r g b
                    has_rgb = any(field.lower() in ['r', 'g', 'b', 'rgb'] for field in fields)
                    if has_rgb:
                        return True
                    else:
                        # Không log để tránh spam
                        return False
                
                # Nếu không tìm thấy FIELDS, thử kiểm tra bằng SIZE
                # PointXYZRGB: SIZE 4 4 4 4 (x, y, z, rgb) = 16 bytes
                # PointXYZI: SIZE 4 4 4 4 (x, y, z, intensity) = 16 bytes
                # PointXYZ: SIZE 4 4 4 = 12 bytes
                size_line = None
                for line in header_lines:
                    if line.startswith('SIZE'):
                        size_line = line
                        break
                
                if size_line:
                    sizes = size_line.split()[1:]
                    try:
                        # Đếm số lượng fields
                        num_fields = len(fields) if fields else len(sizes)
                        # PointXYZRGB thường có 4 fields (x, y, z, rgb) với tổng size >= 16
                        total_size = sum(int(s) for s in sizes if s.isdigit())
                        # Nếu có 4 fields và tổng size >= 16, có thể là RGB
                        # Nhưng cần cẩn thận vì PointXYZI cũng có 4 fields
                        # Tốt nhất là chỉ dựa vào FIELDS line
                        if num_fields >= 4 and total_size >= 16:
                            # Không thể chắc chắn nếu không có FIELDS, nên return False
                            pass
                    except (ValueError, IndexError):
                        pass
                
                return False
            except Exception as e:
                # Lỗi khi parse header - return False để bỏ qua file này
                return False
        except (IOError, OSError, MemoryError) as e:
            # Lỗi khi đọc file - có thể do file quá lớn hoặc lỗi I/O
            return False
        except Exception as e:
            # Bất kỳ lỗi nào khác - return False để tránh crash
            return False
    
    def auto_find_rgb_pcd(self):
        """Tự động tìm file PCD có màu sắc RGB trong nguồn được chọn"""
        # Lấy nguồn được chọn
        source = self.pcd_source_var.get() if hasattr(self, 'pcd_source_var') else "default"
        
        self.log("🔍 Đang tìm file PCD có màu sắc RGB...")
        
        # Thư mục external drive
        external_drive_path = Path("/media/an/ANHSON")
        
        # Nếu chọn nguồn ANHSON, không tự động tìm - người dùng sẽ chọn file qua dialog
        if source == "anhsong":
            # Nếu đã có file được chọn, không cần tìm nữa
            if self.pcd_path_var.get():
                return None
            
            self.log("💡 Vui lòng chọn file PCD bằng nút 'Chọn File'")
            self.info_label.config(text="Vui lòng chọn file PCD từ ANHSON")
            return None
            
        
        # Nếu chọn nguồn default, chỉ tìm trong đường dẫn mặc định
        if not self.default_pcd_dir.exists():
            self.log(f"⚠️  Thư mục PCD không tồn tại: {self.default_pcd_dir}")
            self.info_label.config(text="Thư mục PCD không tồn tại")
            return None
        
        # Danh sách các file ưu tiên tìm (ưu tiên raw points trước)
        priority_files = [
            self.default_pcd_dir / "all_raw_points.pcd",  # Ưu tiên file raw (nhiều điểm nhất)
            self.default_pcd_dir / "all_downsampled_points.pcd",
        ]
        
        # Tìm trong thư mục pcd/ nếu có (thư mục chứa các scan riêng lẻ)
        # Thử nhiều đường dẫn có thể
        possible_map_dirs = [
            self.fast_livo_path / "Log" / "map",
            Path(self.fast_livo_path.parent.parent) / "Log" / "map",
            self.workspace_path.parent / "Log" / "map",
        ]
        pcd_scan_dir = None
        for map_dir in possible_map_dirs:
            test_pcd_dir = map_dir / "pcd"
            if test_pcd_dir.exists():
                pcd_scan_dir = test_pcd_dir
                break
        
        # Kiểm tra các file ưu tiên trước - với bảo vệ chống crash
        for pcd_file in priority_files:
            try:
                if not pcd_file.exists():
                    continue
                
                # Skip file lớn hơn 100MB để tránh crash
                file_size_mb = self.get_file_size_mb(str(pcd_file))
                if file_size_mb > 100:  # Skip file > 100MB
                    continue
                
                if self.check_pcd_has_rgb(str(pcd_file)):
                    self.pcd_path_var.set(str(pcd_file))
                    threading.Thread(target=self._update_pcd_info_safe, args=(str(pcd_file),), daemon=True).start()
                    self.log(f"✅ Đã tự động chọn file có màu: {pcd_file.name}")
                    return str(pcd_file)
            except Exception as e:
                # Bỏ qua file lỗi và tiếp tục
                continue
        
        # Tìm trong thư mục pcd/ (các scan riêng lẻ) - với giới hạn và bảo vệ
        if pcd_scan_dir and pcd_scan_dir.exists():
            try:
                self.log(f"🔍 Tìm trong thư mục: {pcd_scan_dir}")
                pcd_files = list(pcd_scan_dir.glob("*.pcd"))
                # Giới hạn số file check để tránh crash
                max_files_to_check = 20
                pcd_files = sorted(pcd_files[:max_files_to_check], key=lambda x: x.stat().st_mtime if x.exists() else 0, reverse=True)
                
                for pcd_file in pcd_files:
                    try:
                        # Skip file lớn hơn 100MB
                        file_size_mb = self.get_file_size_mb(str(pcd_file))
                        if file_size_mb > 100:  # Skip file > 100MB
                            continue
                        
                        if self.check_pcd_has_rgb(str(pcd_file)):
                            self.pcd_path_var.set(str(pcd_file))
                            threading.Thread(target=self._update_pcd_info_safe, args=(str(pcd_file),), daemon=True).start()
                            self.log(f"✅ Đã tự động chọn file có màu: {pcd_file.name}")
                            return str(pcd_file)
                    except Exception:
                        # Bỏ qua file lỗi
                        continue
            except Exception as e:
                self.log(f"⚠️  Lỗi khi tìm trong thư mục pcd: {e}")
        
        # Nếu không tìm thấy file có RGB, thử tìm bất kỳ file nào nhỏ hơn 100MB
        self.log("⚠️  Không tìm thấy file PCD có màu sắc RGB")
        
        # Thử tìm trong các file ưu tiên mặc định - chỉ chọn file < 100MB
        for pcd_file in priority_files:
            try:
                if not pcd_file.exists():
                    continue
                
                # Chỉ chọn file < 100MB để tránh crash
                file_size_mb = self.get_file_size_mb(str(pcd_file))
                if file_size_mb > 100:  # Skip file > 100MB
                    self.log(f"⚠️  Bỏ qua file quá lớn: {pcd_file.name} ({file_size_mb:.2f} MB)")
                    continue
                
                self.pcd_path_var.set(str(pcd_file))
                threading.Thread(target=self._update_pcd_info_safe, args=(str(pcd_file),), daemon=True).start()
                self.log(f"⚠️  Đã chọn file (có thể không có màu): {pcd_file.name}")
                def update_info():
                    self.info_label.config(text=f"⚠️ File có thể không có màu: {pcd_file.name}")
                self.after(0, update_info)
                return str(pcd_file)
            except Exception:
                continue
        
        self.log(f"❌ Không tìm thấy PCD file trong {self.default_pcd_dir}")
        self.info_label.config(text="Không tìm thấy PCD file")
        return None    
    
    def _update_pcd_info_safe(self, pcd_path):
        """Wrapper an toàn cho _update_pcd_info - chạy trong thread riêng"""
        try:
            self._update_pcd_info(pcd_path)
        except Exception as e:
            self.log(f"❌ Lỗi khi cập nhật thông tin file: {e}")
            def update_error():
                self.info_label.config(text=f"Lỗi khi đọc thông tin file: {e}")
            self.after(0, update_error)
    
    def _update_pcd_info(self, pcd_path):
        """Cập nhật thông tin về PCD file - an toàn với try-catch"""
        try:
            pcd_path_obj = Path(pcd_path)
            if not pcd_path_obj.exists():
                def update_not_exists():
                    self.info_label.config(text="PCD file không tồn tại")
                self.after(0, update_not_exists)
                return
            
            # Lấy thông tin file - sử dụng get_file_size_mb để an toàn
            file_size_mb = self.get_file_size_mb(pcd_path)
            if file_size_mb == 0:
                def update_error():
                    self.info_label.config(text="Không thể đọc thông tin file")
                self.after(0, update_error)
                return
            
            # Nếu file quá lớn, chỉ hiển thị thông tin cơ bản để tránh crash
            if file_size_mb > 1000:  # > 1GB - chỉ hiển thị thông tin cơ bản
                def update_large_file():
                    self.info_label.config(
                        text=f"File: {pcd_path_obj.name}\n"
                             f"Kích thước: {file_size_mb:.2f} MB ({file_size_mb/1024:.2f} GB)\n"
                             f"⚠️ File rất lớn - cần downsample trước khi load"
                    )
                self.after(0, update_large_file)
                return
            
            file_size_gb = file_size_mb / 1024
            
            # Kiểm tra và cảnh báo
            is_too_large, _ = self.is_file_too_large(pcd_path, skip_if_large=False)
            warning_text = ""
            if is_too_large:
                warning_text = f"\n❌ CẢNH BÁO: File quá lớn ({file_size_mb:.2f} MB) - CÓ THỂ GÂY CRASH!"
            elif file_size_mb > self.warning_file_size_mb:
                warning_text = f"\n⚠️ File lớn ({file_size_mb:.2f} MB) - có thể gây crash hoặc treo"
            elif file_size_mb > self.safe_file_size_mb:
                warning_text = f"\n⚠️ File khá lớn ({file_size_mb:.2f} MB) - cẩn thận khi load"
            
            # Kiểm tra memory (không block)
            memory_available = self.check_memory_available()
            memory_warning = ""
            if memory_available is not None:
                # Với file lớn, cần nhiều memory hơn
                required_memory = file_size_mb / 1024 * 2  # Cần ít nhất 2x file size
                if memory_available < required_memory:
                    memory_warning = f"\n⚠️ Memory có thể không đủ (Cần: {required_memory:.2f} GB, Có: {memory_available:.2f} GB)"
            
            # Cập nhật UI trong main thread
            def update_ui():
                try:
                    self.info_label.config(
                        text=f"File: {pcd_path_obj.name}\n"
                             f"Kích thước: {file_size_mb:.2f} MB ({file_size_gb:.2f} GB){warning_text}{memory_warning}\n"
                             f"Đường dẫn: {pcd_path}"
                    )
                except Exception:
                    pass  # Ignore UI update errors
            
            self.after(0, update_ui)
            
        except (OSError, IOError, PermissionError) as e:
            def update_error():
                self.info_label.config(text=f"Lỗi truy cập file: {e}")
            self.after(0, update_error)
        except Exception as e:
            def update_error():
                self.info_label.config(text=f"Lỗi: {e}")
            self.after(0, update_error)
    
    def start_viewer(self):
        """Bắt đầu PCD viewer"""
        if self.is_viewer_running:
            messagebox.showwarning("Cảnh báo", "PCD viewer đang chạy, vui lòng dừng trước")
            return
        
        # Chạy validation và start trong thread riêng để không block UI
        threading.Thread(target=self._start_viewer_worker, daemon=True).start()
    
    def _start_viewer_worker(self):
        """Worker thread để start viewer - không block UI"""
        try:
            # Kiểm tra PCD path
            pcd_path = self.pcd_path_var.get()
            if not pcd_path:
                # Tự động tìm lại file nếu chưa có
                self.auto_find_rgb_pcd()
                pcd_path = self.pcd_path_var.get()
                if not pcd_path:
                    def show_error():
                        messagebox.showerror("Lỗi", "Không tìm thấy file PCD có màu sắc")
                    self.after(0, show_error)
                    return
            
            pcd_path_obj = Path(pcd_path)
            if not pcd_path_obj.exists():
                def show_error():
                    messagebox.showerror("Lỗi", f"PCD file không tồn tại: {pcd_path}")
                self.after(0, show_error)
                return
            
            # Kiểm tra kích thước file và memory trước khi chạy
            is_too_large, file_size_mb = self.is_file_too_large(pcd_path, skip_if_large=False)
            file_size_gb = file_size_mb / 1024
            
            # Tự động downsample nếu file lớn và option được bật
            if is_too_large and self.auto_downsample_var.get() if hasattr(self, 'auto_downsample_var') else False:
                if not HAS_OPEN3D:
                    # Sử dụng callback để không block
                    self._ask_continue_no_open3d(file_size_mb, pcd_path)
                    return
                else:
                    # Hỏi người dùng: đợi downsample hay start ngay với file gốc
                    self._ask_downsample_or_start_immediately(file_size_mb, pcd_path)
                    return
        
            elif is_too_large:
                self._ask_continue_large_file(file_size_mb, pcd_path, None)
                return
            
            # Cảnh báo cho file > 200MB nhưng < 300MB
            elif file_size_mb > self.warning_file_size_mb:
                self._ask_continue_warning_file(file_size_mb, pcd_path)
                return
            
            # Kiểm tra memory
            memory_available = self.check_memory_available()
            if memory_available is not None:
                if memory_available < file_size_gb * 1.5:  # Cần ít nhất 1.5x file size
                    self._ask_continue_low_memory(memory_available, file_size_gb, pcd_path)
                    return
            
            # Tất cả checks đã pass - start viewer
            self._do_start_viewer(pcd_path)
            
        except Exception as e:
            self.log(f"❌ Lỗi trong start viewer worker: {e}")
            def show_error():
                messagebox.showerror("Lỗi", f"Lỗi khi khởi động viewer: {e}")
            self.after(0, show_error)
    
    def _ask_downsample_or_start_immediately(self, file_size_mb, pcd_path):
        """Hỏi người dùng: đợi downsample hay start ngay với file gốc"""
        response_queue = queue.Queue()
        def ask_response():
            response = messagebox.askyesno(
                "📉 File lớn - Chọn hành động",
                f"File PCD lớn ({file_size_mb:.2f} MB)!\n\n"
                f"Bạn muốn:\n\n"
                f"• YES: Đợi downsample file (an toàn, mất thời gian ~1-5 phút)\n"
                f"• NO: Start ngay với file gốc (nhanh nhưng có thể crash!)\n\n"
                f"Khuyến nghị: Đợi downsample để tránh crash."
            )
            response_queue.put(response)
        
        self.after(0, ask_response)
        try:
            wait_for_downsample = response_queue.get(timeout=60)
            if wait_for_downsample:
                # Đợi downsample
                self._do_downsample_and_start(pcd_path)
            else:
                # Start ngay với file gốc
                self.log(f"⚠️  Bỏ qua downsample - Start ngay với file gốc ({file_size_mb:.2f} MB)")
                self._do_start_viewer(pcd_path)
        except queue.Empty:
            self.log("❌ Timeout khi đợi response - Start ngay với file gốc")
            self._do_start_viewer(pcd_path)
    
    def _do_downsample_and_start(self, pcd_path):
        """Thực hiện downsample và sau đó start viewer"""
        try:
            voxel_size = float(self.voxel_size_var.get() if hasattr(self, 'voxel_size_var') else "0.15")
        except ValueError:
            voxel_size = 0.15
        
        file_size_mb = self.get_file_size_mb(pcd_path)
        self.log(f"📉 Đang downsample file ({file_size_mb:.2f} MB) với voxel_size={voxel_size}m...")
        self.log("⏳ Vui lòng đợi, quá trình này có thể mất vài phút...")
        
        def update_status():
            self.status_label.config(
                text="Trạng thái: Đang downsample file (có thể mất vài phút)...",
                foreground="orange"
            )
        self.after(0, update_status)
        
        # Downsample trong thread riêng để không block
        def downsample_worker():
            try:
                downsampled_path = self.downsample_pcd_file(pcd_path, voxel_size=voxel_size)
                if downsampled_path:
                    downsampled_size_mb = self.get_file_size_mb(downsampled_path)
                    self.log(f"✅ Downsample hoàn thành: {Path(downsampled_path).name} ({downsampled_size_mb:.2f} MB)")
                    # Start viewer với file đã downsample
                    self._do_start_viewer(downsampled_path)
                else:
                    self.log("❌ Downsample thất bại")
                    self._ask_continue_after_downsample_error(pcd_path)
            except Exception as e:
                self.log(f"❌ Lỗi khi downsample: {e}")
                self._ask_continue_after_downsample_error(pcd_path)
        
        threading.Thread(target=downsample_worker, daemon=True).start()
    
    def _ask_continue_no_open3d(self, file_size_mb, pcd_path):
        """Hỏi người dùng có muốn tiếp tục khi không có open3d"""
        response_queue = queue.Queue()
        def ask_response():
            response = messagebox.askyesno(
                "⚠️ CẢNH BÁO - File quá lớn!",
                f"File PCD rất lớn ({file_size_mb:.2f} MB)!\n\n"
                f"open3d chưa được cài đặt, không thể tự động downsample.\n\n"
                f"Cài đặt: pip3 install open3d\n\n"
                f"Bạn vẫn muốn tiếp tục với file gốc? (Có thể bị crash!)"
            )
            response_queue.put(response)
        
        self.after(0, ask_response)
        try:
            response = response_queue.get(timeout=60)
            if response:
                self._do_start_viewer(pcd_path)
            else:
                self.log("❌ Người dùng đã hủy")
        except queue.Empty:
            self.log("❌ Timeout khi đợi response")
    
    def _ask_continue_after_downsample_error(self, pcd_path):
        """Hỏi người dùng có muốn tiếp tục sau khi downsample lỗi"""
        response_queue = queue.Queue()
        def ask_response():
            response = messagebox.askyesno(
                "⚠️ Lỗi khi downsample",
                f"Không thể downsample file. Bạn có muốn tiếp tục với file gốc không?\n"
                f"(Có thể bị crash!)"
            )
            response_queue.put(response)
        
        self.after(0, ask_response)
        try:
            response = response_queue.get(timeout=60)
            if response:
                self._do_start_viewer(pcd_path)
            else:
                self.log("❌ Người dùng đã hủy")
        except queue.Empty:
            self.log("❌ Timeout khi đợi response")
    
    def _ask_continue_large_file(self, file_size_mb, pcd_path, downsampled_path):
        """Hỏi người dùng có muốn tiếp tục với file quá lớn"""
        response_queue = queue.Queue()
        def ask_response():
            response = messagebox.askyesno(
                "⚠️ CẢNH BÁO - File quá lớn có thể gây CRASH!",
                f"File PCD rất lớn ({file_size_mb:.2f} MB) có thể gây crash client!\n\n"
                f"Giới hạn an toàn: {self.max_file_size_mb} MB\n"
                f"File hiện tại: {file_size_mb:.2f} MB\n\n"
                f"⚠️ KHÔNG KHUYẾN NGHỊ tiếp tục!\n"
                f"Khuyến nghị: Bật 'Tự động downsample' để giảm kích thước file.\n\n"
                f"Bạn vẫn muốn tiếp tục? (Có thể bị crash!)"
            )
            response_queue.put(response)
        
        self.after(0, ask_response)
        try:
            response = response_queue.get(timeout=60)
            if response:
                self.log(f"⚠️  CẢNH BÁO: Người dùng đã chọn tiếp tục với file lớn ({file_size_mb:.2f} MB)")
                self._do_start_viewer(pcd_path)
            else:
                self.log("❌ Người dùng đã hủy do file quá lớn")
        except queue.Empty:
            self.log("❌ Timeout khi đợi response")
    
    def _ask_continue_warning_file(self, file_size_mb, pcd_path):
        """Hỏi người dùng có muốn tiếp tục với file lớn"""
        response_queue = queue.Queue()
        def ask_response():
            response = messagebox.askyesno(
                "⚠️ Cảnh báo - File lớn",
                f"File PCD lớn ({file_size_mb:.2f} MB) có thể gây crash hoặc treo hệ thống!\n\n"
                f"Kích thước an toàn: < {self.safe_file_size_mb} MB\n"
                f"File hiện tại: {file_size_mb:.2f} MB\n\n"
                f"Bạn có muốn tiếp tục không?\n"
                f"Khuyến nghị: Sử dụng file nhỏ hơn nếu có thể."
            )
            response_queue.put(response)
        
        self.after(0, ask_response)
        try:
            response = response_queue.get(timeout=60)
            if response:
                # Kiểm tra memory trước khi tiếp tục
                file_size_gb = file_size_mb / 1024
                memory_available = self.check_memory_available()
                if memory_available is not None and memory_available < file_size_gb * 1.5:
                    self._ask_continue_low_memory(memory_available, file_size_gb, pcd_path)
                else:
                    self._do_start_viewer(pcd_path)
            else:
                self.log("❌ Người dùng đã hủy do file lớn")
        except queue.Empty:
            self.log("❌ Timeout khi đợi response")
    
    def _ask_continue_low_memory(self, memory_available, file_size_gb, pcd_path):
        """Hỏi người dùng có muốn tiếp tục với memory thấp"""
        response_queue = queue.Queue()
        def ask_response():
            response = messagebox.askyesno(
                "⚠️ Cảnh báo - Memory thấp",
                f"Memory available: {memory_available:.2f} GB\n"
                f"File size: {file_size_gb:.2f} GB\n\n"
                f"Memory có thể không đủ để load file này.\n"
                f"Bạn có muốn tiếp tục không?"
            )
            response_queue.put(response)
        
        self.after(0, ask_response)
        try:
            response = response_queue.get(timeout=60)
            if response:
                self._do_start_viewer(pcd_path)
            else:
                self.log("❌ Người dùng đã hủy do memory không đủ")
        except queue.Empty:
            self.log("❌ Timeout khi đợi response")
    
    def _do_start_viewer(self, pcd_path):
        """Thực hiện start viewer với file đã được validate"""
        try:
            pcd_path_obj = Path(pcd_path)
            
            # Kiểm tra ws setup.sh
            ws_setup = self.workspace_path / "install" / "setup.sh"
            if not ws_setup.exists():
                def show_error():
                    messagebox.showerror(
                        "Lỗi",
                        f"Không tìm thấy ws/install/setup.sh tại: {ws_setup}\n"
                        "Vui lòng build workspace trước."
                    )
                self.after(0, show_error)
                return
            
            # Lấy các parameters
            topic_name = self.topic_var.get().strip() or "/pcd_map"
            frame_id = self.frame_id_var.get().strip() or "map"
            
            try:
                publish_rate = float(self.rate_var.get())
                if publish_rate < 0:
                    publish_rate = 1.0
            except ValueError:
                publish_rate = 1.0
                self.log("⚠️  Giá trị rate không hợp lệ, sử dụng mặc định 1.0")
            
            loop = self.loop_var.get()
            use_rviz = self.use_rviz_var.get()
            
            # Build command
            ros2_setup = "/opt/ros/jazzy/setup.bash"
            
            # Build launch command
            launch_cmd = (
                f"ros2 launch fast_livo pcd_viewer.launch.py "
                f"pcd_file:={pcd_path} "
                f"topic_name:={topic_name} "
                f"frame_id:={frame_id} "
                f"publish_rate:={publish_rate} "
                f"loop:={str(loop).lower()} "
                f"use_rviz:={str(use_rviz).lower()}"
            )
            
            # Source environment
            cmd = (
                f"source {ros2_setup} && "
                f"source {ws_setup} && "
                f"{launch_cmd}"
            )
            
            self.log(f"📝 Bắt đầu PCD viewer")
            self.log(f"📁 PCD file: {pcd_path_obj.name}")
            self.log(f"📁 Đường dẫn: {pcd_path}")
            self.log(f"⚙️  Topic: {topic_name}")
            self.log(f"⚙️  Frame ID: {frame_id}")
            self.log(f"⚙️  Publish Rate: {publish_rate} Hz")
            self.log(f"⚙️  Loop: {'Có' if loop else 'Không'}")
            self.log(f"⚙️  RViz: {'Có' if use_rviz else 'Không'}")
            
            # Sử dụng env để đảm bảo clean environment
            env = os.environ.copy()
            if 'ROS_DOMAIN_ID' not in env:
                env['ROS_DOMAIN_ID'] = '0'
            
            self.pcd_viewer_process = subprocess.Popen(
                cmd,
                shell=True,
                executable="/bin/bash",
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                universal_newlines=True,
                bufsize=1,
                env=env
            )
            
            self.is_viewer_running = True
            self.pcd_file_path = pcd_path
            
            # Update UI trong main thread
            def update_ui():
                self.start_btn.config(state=tk.DISABLED)
                self.stop_btn.config(state=tk.NORMAL)
                self.status_label.config(
                    text="Trạng thái: Đang chạy...",
                    foreground="orange"
                )
            
            self.after(0, update_ui)
            
            # Start thread để đọc output
            threading.Thread(target=self.monitor_viewer_process, daemon=True).start()
            
            self.log("✅ PCD viewer đã được khởi động")
            
        except Exception as e:
            error_msg = f"Không thể bắt đầu PCD viewer: {e}"
            self.log(f"❌ Lỗi: {error_msg}")
            
            def show_error():
                messagebox.showerror("Lỗi", error_msg)
                self.is_viewer_running = False
                self.start_btn.config(state=tk.NORMAL)
                self.stop_btn.config(state=tk.DISABLED)
            
            self.after(0, show_error)
    
    def stop_viewer(self):
        """Dừng PCD viewer"""
        if not self.is_viewer_running:
            return
        
        if self.pcd_viewer_process:
            try:
                self.log("Đang dừng PCD viewer...")
                # Terminate process
                self.pcd_viewer_process.terminate()
                try:
                    self.pcd_viewer_process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    self.pcd_viewer_process.kill()
                    self.pcd_viewer_process.wait()
                
                # Kill các process con (rviz, pcd_viewer_node) nếu còn chạy
                try:
                    # Tìm và kill rviz2 process
                    subprocess.run(
                        ["pkill", "-f", "rviz2.*pcd_viewer"],
                        timeout=2,
                        capture_output=True
                    )
                    # Tìm và kill pcd_viewer_node
                    subprocess.run(
                        ["pkill", "-f", "pcd_viewer_node"],
                        timeout=2,
                        capture_output=True
                    )
                except Exception:
                    pass  # Ignore errors khi kill processes
                    
            except Exception as e:
                self.log(f"Lỗi khi dừng PCD viewer: {e}")
            
            self.pcd_viewer_process = None
        
        self.is_viewer_running = False
        self.is_rviz_running = False
        self.start_btn.config(state=tk.NORMAL)
        self.stop_btn.config(state=tk.DISABLED)
        self.status_label.config(
            text="Trạng thái: Đã dừng",
            foreground="green"
        )
        
        if self.pcd_file_path:
            pcd_name = Path(self.pcd_file_path).name
            self.info_label.config(
                text=f"Đã dừng viewer\nFile: {pcd_name}"
            )
            self.log(f"✅ PCD viewer đã dừng. File: {pcd_name}")
        else:
            self.info_label.config(text="Đã dừng viewer")
            self.log("✅ PCD viewer đã dừng")
    
    def monitor_viewer_process(self):
        """Monitor viewer process output"""
        if not self.pcd_viewer_process:
            return
        
        try:
            for line in iter(self.pcd_viewer_process.stdout.readline, ''):
                if not line:
                    break
                line = line.strip()
                if line:
                    # Log output
                    if any(keyword in line.lower() for keyword in ['error', 'fatal', 'exception', 'failed']):
                        self.log(f"❌ ERROR: {line}")
                    elif any(keyword in line.lower() for keyword in ['warning', 'warn']):
                        self.log(f"⚠️  WARNING: {line}")
                    else:
                        # Log các dòng quan trọng
                        if any(keyword in line.lower() for keyword in ['loaded', 'publishing', 'started', 'rviz']):
                            self.log(line)
        except Exception as e:
            self.log(f"Lỗi khi đọc output: {e}")
        
        # Kiểm tra exit code
        if self.pcd_viewer_process.poll() is not None:
            exit_code = self.pcd_viewer_process.poll()
            if exit_code != 0:
                self.log(f"✗ PCD viewer đã dừng với exit code: {exit_code}")
            else:
                self.log(f"✓ PCD viewer đã hoàn thành")
            
            self.is_viewer_running = False
            self.after(0, partial(self._update_viewer_stopped))
    
    def _update_viewer_stopped(self):
        """Helper function để update UI sau khi viewer dừng"""
        try:
            self.start_btn.config(state=tk.NORMAL)
            self.stop_btn.config(state=tk.DISABLED)
            self.status_label.config(
                text="Trạng thái: Đã dừng",
                foreground="green"
            )
            if self.pcd_file_path:
                pcd_name = Path(self.pcd_file_path).name
                self.info_label.config(
                    text=f"Viewer đã dừng\nFile: {pcd_name}"
                )
        except Exception as e:
            print(f"Lỗi khi update UI: {e}")

