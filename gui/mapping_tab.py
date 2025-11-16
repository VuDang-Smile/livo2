#!/usr/bin/env python3
"""
Mapping Tab Module
Chứa MappingTab để start/stop FAST-LIVO2 mapping
"""

import threading
import subprocess
from pathlib import Path
from datetime import datetime
import os
import platform
import yaml
import shutil

try:
    import tkinter as tk
    from tkinter import ttk, messagebox, scrolledtext, filedialog
except ImportError as e:
    print(f"Lỗi import: {e}")
    import sys
    sys.exit(1)


class MappingTab(ttk.Frame):
    """Tab cho FAST-LIVO2 Mapping"""
    
    def __init__(self, parent):
        super().__init__(parent)
        
        # Paths
        self.workspace_path = Path(__file__).parent.parent / "ws"
        self.drive_ws_path = Path(__file__).parent.parent / "drive_ws"
        
        # Processes
        self.mapping_process = None
        self.rviz_process = None
        
        # State
        self.is_mapping_running = False
        self.is_rviz_running = False
        self.selected_config = "mid360_perspective"  # Default
        self.output_path = None
        self.calibration_file_path = None
        
        # Tạo UI
        self.create_widgets()
        
        # Cập nhật thông tin output ban đầu
        self.update_output_info()
    
    def create_widgets(self):
        """Tạo các widget cho tab Mapping"""
        
        # Title
        title_label = ttk.Label(
            self,
            text="FAST-LIVO2 Mapping",
            font=("Arial", 16, "bold")
        )
        title_label.pack(pady=10)
        
        # Frame điều khiển
        control_frame = ttk.Frame(self, padding="10")
        control_frame.pack(fill=tk.X)
        
        # Frame chọn config
        config_frame = ttk.LabelFrame(control_frame, text="Cấu hình", padding="5")
        config_frame.pack(side=tk.LEFT, padx=5, fill=tk.X, expand=True)
        
        self.config_var = tk.StringVar(value="mid360_perspective")
        ttk.Radiobutton(
            config_frame,
            text="MID360 Perspective",
            variable=self.config_var,
            value="mid360_perspective",
            command=self.on_config_change
        ).pack(side=tk.LEFT, padx=5)
        
        ttk.Radiobutton(
            config_frame,
            text="Avia Perspective",
            variable=self.config_var,
            value="avia_perspective",
            command=self.on_config_change
        ).pack(side=tk.LEFT, padx=5)
        
        # Frame chọn file calibration
        calib_frame = ttk.LabelFrame(self, text="File Calibration", padding="10")
        calib_frame.pack(fill=tk.X, padx=10, pady=5)
        
        calib_path_frame = ttk.Frame(calib_frame)
        calib_path_frame.pack(fill=tk.X, padx=5, pady=5)
        
        ttk.Label(
            calib_path_frame,
            text="Đường dẫn:",
            font=("Arial", 10)
        ).pack(side=tk.LEFT, padx=5)
        
        self.calibration_path_var = tk.StringVar(value="")
        self.calibration_entry = ttk.Entry(
            calib_path_frame,
            textvariable=self.calibration_path_var,
            width=60,
            font=("Arial", 9)
        )
        self.calibration_entry.pack(side=tk.LEFT, padx=5, fill=tk.X, expand=True)
        
        ttk.Button(
            calib_path_frame,
            text="Browse",
            command=self.browse_calibration_file
        ).pack(side=tk.LEFT, padx=5)
        
        self.apply_calib_btn = ttk.Button(
            calib_path_frame,
            text="Apply Calibration",
            command=self.apply_calibration,
            state=tk.DISABLED
        )
        self.apply_calib_btn.pack(side=tk.LEFT, padx=5)
        
        # Nút Start Mapping
        self.start_mapping_btn = ttk.Button(
            control_frame,
            text="Start Mapping",
            command=self.start_mapping,
            state=tk.NORMAL
        )
        self.start_mapping_btn.pack(side=tk.LEFT, padx=5)
        
        # Nút Stop Mapping
        self.stop_mapping_btn = ttk.Button(
            control_frame,
            text="Stop Mapping",
            command=self.stop_mapping,
            state=tk.DISABLED
        )
        self.stop_mapping_btn.pack(side=tk.LEFT, padx=5)
        
        # Nút Start RViz
        self.start_rviz_btn = ttk.Button(
            control_frame,
            text="Start RViz",
            command=self.start_rviz,
            state=tk.NORMAL
        )
        self.start_rviz_btn.pack(side=tk.LEFT, padx=5)
        
        # Nút Stop RViz
        self.stop_rviz_btn = ttk.Button(
            control_frame,
            text="Stop RViz",
            command=self.stop_rviz,
            state=tk.DISABLED
        )
        self.stop_rviz_btn.pack(side=tk.LEFT, padx=5)
        
        # Label trạng thái
        self.status_label = ttk.Label(
            control_frame,
            text="Trạng thái: Sẵn sàng",
            foreground="green"
        )
        self.status_label.pack(side=tk.LEFT, padx=20)
        
        # Frame thông tin kết quả đầu ra
        output_frame = ttk.LabelFrame(self, text="Kết quả đầu ra", padding="10")
        output_frame.pack(fill=tk.X, padx=10, pady=10)
        
        # Output path label
        path_frame = ttk.Frame(output_frame)
        path_frame.pack(fill=tk.X, padx=5, pady=2)
        
        self.output_path_label = ttk.Label(
            path_frame,
            text="Đường dẫn kết quả: Chưa có",
            font=("Arial", 10),
            foreground="gray"
        )
        self.output_path_label.pack(side=tk.LEFT, anchor=tk.W)
        
        # Nút mở thư mục kết quả
        self.open_output_btn = ttk.Button(
            path_frame,
            text="📂 Mở thư mục",
            command=self.open_output_folder,
            state=tk.DISABLED
        )
        self.open_output_btn.pack(side=tk.LEFT, padx=5)
        
        # Nút copy đường dẫn
        self.copy_path_btn = ttk.Button(
            path_frame,
            text="📋 Copy đường dẫn",
            command=self.copy_output_path,
            state=tk.DISABLED
        )
        self.copy_path_btn.pack(side=tk.LEFT, padx=5)
        
        # Output files info
        self.output_files_label = ttk.Label(
            output_frame,
            text="",
            font=("Arial", 9),
            foreground="gray"
        )
        self.output_files_label.pack(anchor=tk.W, padx=5, pady=2)
        
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
    
    def log(self, message):
        """Thêm message vào log"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.log_text.config(state=tk.NORMAL)
        self.log_text.insert(tk.END, f"[{timestamp}] {message}\n")
        self.log_text.see(tk.END)
        self.log_text.config(state=tk.DISABLED)
    
    def on_config_change(self):
        """Xử lý khi thay đổi config"""
        self.selected_config = self.config_var.get()
        self.log(f"Đã chọn config: {self.selected_config}")
    
    def browse_calibration_file(self):
        """Browse để chọn file calibration yaml"""
        initial_dir = self.workspace_path / "calibration_data" / "preprocessed"
        if not initial_dir.exists():
            initial_dir = self.workspace_path / "calibration_data"
        
        file_path = filedialog.askopenfilename(
            title="Chọn file Calibration YAML",
            initialdir=str(initial_dir),
            filetypes=[
                ("YAML files", "*.yaml *.yml"),
                ("All files", "*.*")
            ]
        )
        
        if file_path:
            self.calibration_path_var.set(file_path)
            self.calibration_file_path = file_path
            self.apply_calib_btn.config(state=tk.NORMAL)
            self.log(f"Đã chọn file calibration: {file_path}")
    
    def apply_calibration(self):
        """Apply calibration từ file đã chọn vào mid360_perspective.yaml"""
        if not self.calibration_file_path or not Path(self.calibration_file_path).exists():
            messagebox.showerror("Lỗi", "Vui lòng chọn file calibration trước")
            return
        
        # Chỉ update cho mid360_perspective config
        if self.selected_config != "mid360_perspective":
            messagebox.showwarning(
                "Cảnh báo", 
                "Chức năng này chỉ áp dụng cho config MID360 Perspective.\n"
                "Vui lòng chọn MID360 Perspective trước."
            )
            return
        
        try:
            # Đường dẫn đến file config cần update (cả src và install)
            fast_livo_src_path = self.workspace_path / "src" / "FAST-LIVO2"
            config_file_src = fast_livo_src_path / "config" / "mid360_perspective.yaml"
            
            # Tìm file trong install directory
            install_config_path = self.workspace_path / "install" / "fast_livo" / "share" / "fast_livo" / "config" / "mid360_perspective.yaml"
            
            # Kiểm tra file nào tồn tại
            config_files_to_update = []
            if config_file_src.exists():
                config_files_to_update.append(("src", config_file_src))
            if install_config_path.exists():
                config_files_to_update.append(("install", install_config_path))
            
            if not config_files_to_update:
                messagebox.showerror("Lỗi", f"Không tìm thấy file config trong src hoặc install directory")
                return
            
            # Backup các file gốc
            backup_files = []
            for location, config_file in config_files_to_update:
                backup_file = config_file.with_suffix('.yaml.backup')
                shutil.copy2(config_file, backup_file)
                backup_files.append((location, backup_file))
                self.log(f"✅ Đã backup file config {location}: {backup_file}")
            
            # Sử dụng file đầu tiên để đọc (thường là src)
            config_file = config_files_to_update[0][1]
            
            # Đọc file calibration
            with open(self.calibration_file_path, 'r') as f:
                calib_data = yaml.safe_load(f)
            
            # Extract extrin_calib data
            extrin_calib_data = None
            if 'extrin_calib' in calib_data:
                extrin_calib_data = calib_data['extrin_calib']
            elif 'ros__parameters' in calib_data and 'extrin_calib' in calib_data['ros__parameters']:
                extrin_calib_data = calib_data['ros__parameters']['extrin_calib']
            
            if not extrin_calib_data:
                messagebox.showerror("Lỗi", "Không tìm thấy extrin_calib trong file calibration")
                return
            
            # Đọc file config hiện tại
            with open(config_file, 'r') as f:
                config_text = f.read()
            
            # Parse YAML để lấy cấu trúc
            config_data = yaml.safe_load(config_text)
            
            # Update extrin_calib với dữ liệu mới
            if '/**' in config_data and 'ros__parameters' in config_data['/**']:
                if 'extrin_calib' not in config_data['/**']['ros__parameters']:
                    config_data['/**']['ros__parameters']['extrin_calib'] = {}
                
                # Update Rcl và Pcl
                if 'Rcl' in extrin_calib_data:
                    config_data['/**']['ros__parameters']['extrin_calib']['Rcl'] = extrin_calib_data['Rcl']
                if 'Pcl' in extrin_calib_data:
                    config_data['/**']['ros__parameters']['extrin_calib']['Pcl'] = extrin_calib_data['Pcl']
            
            # Đọc lại file gốc để giữ format và comments
            with open(config_file, 'r') as f:
                original_lines = f.readlines()
            
            # Tìm và thay thế Rcl và Pcl trong original text, giữ nguyên format
            new_lines = []
            i = 0
            rcl_replaced = False
            pcl_replaced = False
            
            while i < len(original_lines):
                line = original_lines[i]
                stripped = line.strip()
                indent = len(line) - len(line.lstrip()) if line.strip() else 0
                
                # Tìm Rcl đầu tiên
                if 'Rcl:' in stripped and not rcl_replaced:
                    rcl_val = extrin_calib_data.get('Rcl', [])
                    if len(rcl_val) == 9:  # 3x3 matrix
                        # Format multi-line như file gốc
                        new_lines.append(f"{' ' * indent}Rcl: [{rcl_val[0]}, {rcl_val[1]}, {rcl_val[2]},\n")
                        new_lines.append(f"{' ' * (indent + 12)}{rcl_val[3]}, {rcl_val[4]}, {rcl_val[5]},\n")
                        new_lines.append(f"{' ' * (indent + 12)}{rcl_val[6]}, {rcl_val[7]}, {rcl_val[8]}]\n")
                    else:
                        new_lines.append(f"{' ' * indent}Rcl: {rcl_val}\n")
                    rcl_replaced = True
                    i += 1
                    # Skip các dòng tiếp theo của Rcl cũ cho đến khi gặp dòng mới có indent <= rcl_indent
                    while i < len(original_lines):
                        next_line = original_lines[i]
                        next_stripped = next_line.strip()
                        next_indent = len(next_line) - len(next_line.lstrip()) if next_line.strip() else 0
                        # Dừng khi gặp dòng mới (không phải comment, không phải tiếp tục array)
                        if next_stripped and (next_indent <= indent) and 'Rcl:' not in next_stripped:
                            break
                        i += 1
                    continue
                
                # Tìm Pcl đầu tiên
                elif 'Pcl:' in stripped and not pcl_replaced:
                    pcl_val = extrin_calib_data.get('Pcl', [])
                    new_lines.append(f"{' ' * indent}Pcl: {pcl_val}\n")
                    pcl_replaced = True
                    i += 1
                    # Skip các dòng tiếp theo của Pcl cũ
                    while i < len(original_lines):
                        next_line = original_lines[i]
                        next_stripped = next_line.strip()
                        next_indent = len(next_line) - len(next_line.lstrip()) if next_line.strip() else 0
                        if next_stripped and (next_indent <= indent) and 'Pcl:' not in next_stripped:
                            break
                        i += 1
                    continue
                
                # Bỏ qua các Rcl hoặc Pcl duplicate
                elif ('Rcl:' in stripped and rcl_replaced) or ('Pcl:' in stripped and pcl_replaced):
                    i += 1
                    # Skip toàn bộ block duplicate
                    while i < len(original_lines):
                        next_line = original_lines[i]
                        next_stripped = next_line.strip()
                        next_indent = len(next_line) - len(next_line.lstrip()) if next_line.strip() else 0
                        if next_stripped and next_indent <= indent:
                            break
                        i += 1
                    continue
                
                # Dòng bình thường
                else:
                    new_lines.append(line)
                    i += 1
            
            # Ghi file mới cho tất cả các file cần update
            updated_files = []
            for location, config_file_path in config_files_to_update:
                with open(config_file_path, 'w') as f:
                    f.writelines(new_lines)
                updated_files.append(f"{location}: {config_file_path}")
                self.log(f"✅ Đã cập nhật file config {location}: {config_file_path}")
            
            if rcl_replaced and pcl_replaced:
                backup_info = "\n".join([f"{loc}: {bf}" for loc, bf in backup_files])
                files_info = "\n".join(updated_files)
                self.log("✅ Đã cập nhật Rcl và Pcl vào mid360_perspective.yaml")
                messagebox.showinfo(
                    "Thành công",
                    f"Đã cập nhật calibration vào:\n{files_info}\n\n"
                    f"File backup:\n{backup_info}\n\n"
                    f"⚠️ Lưu ý: Nếu đã build package, file trong install đã được cập nhật.\n"
                    f"Nếu chưa build, cần build lại để đồng bộ."
                )
            else:
                self.log("⚠️ Không tìm thấy Rcl hoặc Pcl để cập nhật")
                messagebox.showwarning("Cảnh báo", "Không tìm thấy Rcl hoặc Pcl trong file config để cập nhật")
            
        except Exception as e:
            import traceback
            error_msg = f"Lỗi khi apply calibration: {e}"
            self.log(f"❌ {error_msg}")
            self.log(f"Chi tiết: {traceback.format_exc()}")
            messagebox.showerror("Lỗi", error_msg)
    
    def get_output_path(self):
        """Lấy đường dẫn thư mục output của FAST-LIVO2"""
        # FAST-LIVO2 lưu kết quả trong thư mục Log/ của package
        fast_livo_path = self.workspace_path / "src" / "FAST-LIVO2"
        log_path = fast_livo_path / "Log"
        return log_path
    
    def update_output_info(self):
        """Cập nhật thông tin kết quả đầu ra"""
        output_path = self.get_output_path()
        self.output_path = output_path
        
        if output_path.exists():
            self.output_path_label.config(
                text=f"Đường dẫn kết quả: {output_path}",
                foreground="green"
            )
            
            # Kiểm tra các file output
            result_dir = output_path / "result"
            pcd_dir = output_path / "PCD"
            colmap_dir = output_path / "Colmap"
            
            files_info = []
            if result_dir.exists():
                result_files = list(result_dir.glob("*.txt"))
                if result_files:
                    files_info.append(f"Pose files: {len(result_files)}")
            
            if pcd_dir.exists():
                pcd_files = list(pcd_dir.glob("*.pcd"))
                if pcd_files:
                    files_info.append(f"PCD files: {len(pcd_files)}")
            
            if colmap_dir.exists():
                points_file = colmap_dir / "sparse" / "0" / "points3D.txt"
                if points_file.exists():
                    files_info.append("Colmap output: Có")
            
            if files_info:
                self.output_files_label.config(
                    text=" | ".join(files_info),
                    foreground="green"
                )
            else:
                self.output_files_label.config(
                    text="Chưa có file kết quả",
                    foreground="gray"
                )
        else:
            self.output_path_label.config(
                text=f"Đường dẫn kết quả: {output_path} (chưa tồn tại)",
                foreground="orange"
            )
            self.output_files_label.config(
                text="Thư mục sẽ được tạo khi mapping chạy",
                foreground="gray"
            )
        
        # Cập nhật trạng thái nút
        if output_path.exists():
            self.open_output_btn.config(state=tk.NORMAL)
            self.copy_path_btn.config(state=tk.NORMAL)
        else:
            self.open_output_btn.config(state=tk.DISABLED)
            self.copy_path_btn.config(state=tk.DISABLED)
    
    def open_output_folder(self):
        """Mở thư mục kết quả trong file explorer"""
        output_path = self.get_output_path()
        if not output_path.exists():
            messagebox.showwarning("Cảnh báo", f"Thư mục kết quả chưa tồn tại:\n{output_path}")
            return
        
        try:
            system = platform.system()
            if system == "Windows":
                os.startfile(str(output_path))
            elif system == "Darwin":  # macOS
                subprocess.run(["open", str(output_path)])
            else:  # Linux
                subprocess.run(["xdg-open", str(output_path)])
            
            self.log(f"✅ Đã mở thư mục kết quả: {output_path}")
        except Exception as e:
            error_msg = f"Không thể mở thư mục: {e}"
            self.log(f"❌ {error_msg}")
            messagebox.showerror("Lỗi", error_msg)
    
    def copy_output_path(self):
        """Copy đường dẫn kết quả vào clipboard"""
        output_path = self.get_output_path()
        try:
            self.clipboard_clear()
            self.clipboard_append(str(output_path))
            self.log(f"✅ Đã copy đường dẫn vào clipboard: {output_path}")
            messagebox.showinfo("Thành công", f"Đã copy đường dẫn:\n{output_path}")
        except Exception as e:
            error_msg = f"Không thể copy đường dẫn: {e}"
            self.log(f"❌ {error_msg}")
            messagebox.showerror("Lỗi", error_msg)
    
    def start_mapping(self):
        """Start FAST-LIVO2 mapping"""
        if self.is_mapping_running:
            self.log("Mapping đã đang chạy")
            return
        
        try:
            setup_script = self.workspace_path / "install" / "setup.sh"
            if not setup_script.exists():
                messagebox.showerror(
                    "Lỗi",
                    f"Không tìm thấy setup.sh tại: {setup_script}\n"
                    "Vui lòng build workspace trước."
                )
                return
            
            # Chọn launch file dựa trên config
            if self.selected_config == "mid360_perspective":
                launch_file_name = "mapping_mid360_perspective.launch.py"
            else:
                launch_file_name = "mapping_avia_perspective.launch.py"
            
            # Source setup và chạy launch
            # Sử dụng env để đảm bảo unbuffered output
            env = os.environ.copy()
            env['PYTHONUNBUFFERED'] = '1'
            
            cmd = [
                "bash", "-c",
                f"source {setup_script} && "
                f"ros2 launch fast_livo {launch_file_name}"
            ]
            
            self.log(f"Đang khởi động mapping với config: {self.selected_config}")
            self.log(f"Launch file: {launch_file_name}")
            self.log(f"Command: {' '.join(cmd)}")
            
            self.mapping_process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                cwd=str(self.workspace_path),
                bufsize=1,  # Line buffered
                universal_newlines=True,
                env=env
            )
            
            # Đợi một chút để process khởi động
            import time
            time.sleep(0.5)
            
            # Kiểm tra process có đang chạy không
            if self.mapping_process.poll() is not None:
                # Process đã kết thúc ngay lập tức, có thể có lỗi
                error_output = self.mapping_process.stdout.read() if self.mapping_process.stdout else "Không có output"
                self.log(f"❌ Process đã kết thúc ngay với code: {self.mapping_process.returncode}")
                self.log(f"Output: {error_output[:500]}")  # Chỉ hiển thị 500 ký tự đầu
                messagebox.showerror("Lỗi", f"Mapping process đã kết thúc ngay. Kiểm tra log để biết chi tiết.")
                self.mapping_process = None
                return
            
            self.is_mapping_running = True
            self.start_mapping_btn.config(state=tk.DISABLED)
            self.stop_mapping_btn.config(state=tk.NORMAL)
            self.status_label.config(
                text="Trạng thái: 🚀 Mapping đang chạy",
                foreground="green"
            )
            
            # Cập nhật thông tin output path
            self.update_output_info()
            
            # Start thread để đọc output
            threading.Thread(
                target=self._read_mapping_output,
                daemon=True
            ).start()
            
            # Start thread để cập nhật output info định kỳ
            threading.Thread(
                target=self._monitor_output,
                daemon=True
            ).start()
            
            self.log("✅ Mapping đã được khởi động")
            self.log(f"Kết quả sẽ được lưu tại: {self.get_output_path()}")
            self.log("📝 Đang đọc log từ mapping process...")
            
        except Exception as e:
            self.log(f"❌ Lỗi khởi động mapping: {e}")
            messagebox.showerror("Lỗi", f"Không thể khởi động mapping: {e}")
            self.is_mapping_running = False
            self.start_mapping_btn.config(state=tk.NORMAL)
            self.stop_mapping_btn.config(state=tk.DISABLED)
    
    def _read_mapping_output(self):
        """Đọc output từ mapping process"""
        try:
            if not self.mapping_process or not self.mapping_process.stdout:
                return
            
            # Đọc output từ process
            while self.is_mapping_running and self.mapping_process:
                line = self.mapping_process.stdout.readline()
                if not line:
                    # Kiểm tra nếu process đã kết thúc
                    if self.mapping_process.poll() is not None:
                        return_code = self.mapping_process.returncode
                        self.after(0, lambda rc=return_code: self.log(f"⚠️ Mapping process đã kết thúc với code: {rc}"))
                        break
                    # Nếu chưa có output, đợi một chút
                    import time
                    time.sleep(0.1)
                    continue
                
                line = line.strip()
                if line:
                    # Sử dụng lambda với default argument để capture giá trị đúng
                    self.after(0, lambda l=line: self.log(l))
                    
        except Exception as e:
            error_msg = str(e)
            self.after(0, lambda msg=error_msg: self.log(f"Lỗi đọc output: {msg}"))
    
    def _monitor_output(self):
        """Giám sát và cập nhật thông tin output định kỳ"""
        import time
        while self.is_mapping_running:
            time.sleep(5)  # Cập nhật mỗi 5 giây
            if self.is_mapping_running:
                self.after(0, self.update_output_info)
    
    def stop_mapping(self):
        """Stop FAST-LIVO2 mapping"""
        if not self.is_mapping_running:
            return
        
        try:
            if self.mapping_process:
                self.mapping_process.terminate()
                try:
                    self.mapping_process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    self.mapping_process.kill()
                    self.mapping_process.wait()
                self.mapping_process = None
            
            self.is_mapping_running = False
            self.start_mapping_btn.config(state=tk.NORMAL)
            self.stop_mapping_btn.config(state=tk.DISABLED)
            self.status_label.config(
                text="Trạng thái: Mapping đã dừng",
                foreground="orange"
            )
            
            # Cập nhật thông tin output cuối cùng
            self.update_output_info()
            
            self.log("✅ Mapping đã được dừng")
            self.log(f"Kết quả đã được lưu tại: {self.get_output_path()}")
            
        except Exception as e:
            self.log(f"❌ Lỗi dừng mapping: {e}")
            messagebox.showerror("Lỗi", f"Không thể dừng mapping: {e}")
    
    def get_rviz_config_path(self):
        """Lấy đường dẫn đến file config RViz của FAST-LIVO2"""
        # Tìm file config RViz trong thư mục FAST-LIVO2
        fast_livo_path = self.workspace_path / "src" / "FAST-LIVO2"
        rviz_config_file = fast_livo_path / "rviz_cfg" / "fast_livo2.rviz"
        
        if rviz_config_file.exists():
            return str(rviz_config_file)
        
        # Nếu không tìm thấy, thử tìm trong install directory (nếu đã build)
        try:
            from ament_index_python.packages import get_package_share_directory
            rviz_config_file = Path(get_package_share_directory("fast_livo")) / "rviz_cfg" / "fast_livo2.rviz"
            if rviz_config_file.exists():
                return str(rviz_config_file)
        except:
            pass
        
        return None
    
    def start_rviz(self):
        """Start RViz với config sẵn của FAST-LIVO2"""
        if self.is_rviz_running:
            self.log("RViz đã đang chạy")
            return
        
        try:
            setup_script = self.workspace_path / "install" / "setup.sh"
            
            # Tìm file config RViz
            rviz_config_path = self.get_rviz_config_path()
            
            # Tạo clean environment cho RViz để tránh xung đột với OpenCV Qt plugins
            env = os.environ.copy()
            
            # Loại bỏ OpenCV Qt plugins khỏi QT_PLUGIN_PATH
            if 'QT_PLUGIN_PATH' in env:
                paths = env['QT_PLUGIN_PATH'].split(':')
                paths = [p for p in paths if 'cv2' not in p and 'opencv' not in p.lower()]
                if paths:
                    env['QT_PLUGIN_PATH'] = ':'.join(paths)
                else:
                    env.pop('QT_PLUGIN_PATH', None)
            
            # Unset QT_QPA_PLATFORM_PLUGIN_PATH hoàn toàn để Qt tự tìm system plugins
            # Thay vì set về empty, unset hoàn toàn để Qt không bị ảnh hưởng bởi OpenCV
            if 'QT_QPA_PLATFORM_PLUGIN_PATH' in env:
                env.pop('QT_QPA_PLATFORM_PLUGIN_PATH', None)
            
            # Tìm system Qt plugins và set nếu cần
            # RViz2 thường dùng Qt5, nhưng có thể dùng Qt6
            import platform
            if platform.system() == 'Linux':
                # Thử tìm system Qt plugins (ưu tiên Qt5 vì RViz2 thường dùng Qt5)
                possible_paths = [
                    '/usr/lib/x86_64-linux-gnu/qt5/plugins',
                    '/usr/lib/qt5/plugins',
                    '/usr/lib/x86_64-linux-gnu/qt6/plugins',
                    '/usr/lib/qt6/plugins',
                ]
                for path in possible_paths:
                    if os.path.exists(path) and os.path.isdir(path):
                        # Kiểm tra xem có thư mục platforms không
                        platforms_dir = os.path.join(path, 'platforms')
                        if os.path.exists(platforms_dir):
                            env['QT_QPA_PLATFORM_PLUGIN_PATH'] = platforms_dir
                            self.log(f"✅ Tìm thấy system Qt plugins tại: {platforms_dir}")
                            break
            
            # Tạo command để chạy RViz với config
            if rviz_config_path:
                cmd = [
                    "bash", "-c",
                    f"source {setup_script} && rviz2 -d {rviz_config_path}"
                ]
                self.log(f"Đang khởi động RViz với config: {rviz_config_path}")
            else:
                cmd = [
                    "bash", "-c",
                    f"source {setup_script} && rviz2"
                ]
                self.log("Đang khởi động RViz (không có config file)")
                self.log("⚠️ Không tìm thấy file config RViz, sẽ dùng config mặc định")
            
            self.log(f"QT_QPA_PLATFORM_PLUGIN_PATH: {env.get('QT_QPA_PLATFORM_PLUGIN_PATH', 'unset')}")
            
            # Không capture output để RViz có thể hiển thị GUI
            self.rviz_process = subprocess.Popen(
                cmd,
                stdout=None,  # Không capture, để hiển thị terminal
                stderr=None,  # Không capture, để hiển thị terminal
                cwd=str(self.workspace_path),
                env=env,  # Sử dụng clean environment
                start_new_session=True  # Tạo session mới để có thể kill process tree
            )
            
            self.is_rviz_running = True
            self.start_rviz_btn.config(state=tk.DISABLED)
            self.stop_rviz_btn.config(state=tk.NORMAL)
            if rviz_config_path:
                self.log("✅ RViz đã được khởi động với config FAST-LIVO2")
            else:
                self.log("✅ RViz đã được khởi động (cửa sổ RViz sẽ hiển thị riêng)")
            
        except Exception as e:
            self.log(f"❌ Lỗi khởi động RViz: {e}")
            messagebox.showerror("Lỗi", f"Không thể khởi động RViz: {e}")
            self.is_rviz_running = False
            self.start_rviz_btn.config(state=tk.NORMAL)
            self.stop_rviz_btn.config(state=tk.DISABLED)
    
    def stop_rviz(self):
        """Stop RViz"""
        if not self.is_rviz_running:
            return
        
        try:
            if self.rviz_process:
                # Sử dụng kill để đảm bảo dừng process và tất cả child processes
                try:
                    self.rviz_process.terminate()
                    self.rviz_process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    # Nếu terminate không work, dùng kill
                    self.rviz_process.kill()
                    self.rviz_process.wait()
                except ProcessLookupError:
                    # Process đã kết thúc
                    pass
                self.rviz_process = None
            
            self.is_rviz_running = False
            self.start_rviz_btn.config(state=tk.NORMAL)
            self.stop_rviz_btn.config(state=tk.DISABLED)
            self.log("✅ RViz đã được dừng")
            
        except Exception as e:
            self.log(f"❌ Lỗi dừng RViz: {e}")
            messagebox.showerror("Lỗi", f"Không thể dừng RViz: {e}")

