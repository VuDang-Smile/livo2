"""
Logging Module
Quản lý logging và hiển thị log messages trong UI
"""
import tkinter as tk
import threading
import time
from datetime import datetime


class LoggingManager:
    """Quản lý logging cho Bag Mapping Interface"""
    
    def __init__(self, root, log_panel, translator, config):
        """
        Initialize logging manager
        
        Args:
            root: Tkinter root window
            log_panel: Tkinter Text widget để hiển thị log
            translator: Translator instance
            config: BagMappingConfig instance
        """
        self.root = root
        self.log_panel = log_panel
        self.translator = translator
        self.config = config
        
        # Queue system cho smooth updates
        self.log_queue = []
        self.log_update_lock = threading.Lock()
        self.log_update_thread_running = True
        
        # Start log update thread
        threading.Thread(target=self._log_update_worker, daemon=True).start()
    
    def add_log(self, message):
        """Thêm log message vào queue để update mượt hơn"""
        time_str = datetime.now().strftime("%H:%M:%S")
        log_entry = f"[{time_str}] {message}\n"
        
        with self.log_update_lock:
            self.log_queue.append(log_entry)
    
    def add_log_success(self, message_key, **kwargs):
        """Thêm success log message"""
        msg = self.translator.get(message_key, message_key)
        for key, value in kwargs.items():
            msg = msg.replace(f'{{{key}}}', str(value))
        self.add_log(f"✅ {msg}")
    
    def add_log_warning(self, message_key, **kwargs):
        """Thêm warning log message"""
        msg = self.translator.get(message_key, message_key)
        for key, value in kwargs.items():
            msg = msg.replace(f'{{{key}}}', str(value))
        self.add_log(f"⚠️ {msg}")
    
    def clear_log(self):
        """Xóa tất cả log messages"""
        self.log_panel.config(state=tk.NORMAL)
        self.log_panel.delete('1.0', tk.END)
        self.log_panel.config(state=tk.DISABLED)
    
    def _log_update_worker(self):
        """Worker thread để batch update log panel mượt hơn"""
        while self.log_update_thread_running:
            try:
                # Lấy batch log entries từ queue
                batch = []
                with self.log_update_lock:
                    if self.log_queue:
                        batch = self.log_queue[:self.config.log_batch_size]
                        self.log_queue = self.log_queue[self.config.log_batch_size:]
                
                if batch:
                    # Update UI trong main thread
                    self.root.after(0, lambda: self._update_log_panel(batch))
                
                # Sleep ngắn để không chiếm CPU
                time.sleep(0.05)  # 50ms = 20 updates/second
            except Exception as e:
                # Nếu có lỗi, log trực tiếp để không mất thông tin
                try:
                    self.root.after(0, lambda: self._update_log_panel_direct(f"Error in log worker: {e}\n"))
                except:
                    pass
                time.sleep(0.1)
    
    def _update_log_panel(self, batch):
        """Update log panel với batch entries"""
        try:
            self.log_panel.config(state=tk.NORMAL)
            
            # Insert tất cả entries trong batch
            for entry in batch:
                self.log_panel.insert(tk.END, entry)
            
            # Smooth scroll đến cuối
            self.log_panel.see(tk.END)
            
            # Giới hạn số dòng để tránh memory leak (giữ tối đa 1000 dòng)
            line_count = int(self.log_panel.index('end-1c').split('.')[0])
            if line_count > 1000:
                # Xóa 200 dòng đầu
                self.log_panel.delete('1.0', '200.0')
            
            self.log_panel.config(state=tk.DISABLED)
        except Exception as e:
            # Fallback nếu có lỗi
            print(f"Error updating log panel: {e}")
    
    def _update_log_panel_direct(self, message):
        """Update log panel trực tiếp (fallback)"""
        try:
            self.log_panel.config(state=tk.NORMAL)
            self.log_panel.insert(tk.END, message)
            self.log_panel.see(tk.END)
            self.log_panel.config(state=tk.DISABLED)
        except:
            pass
    
    def stop(self):
        """Dừng logging thread"""
        self.log_update_thread_running = False
