"""
Progress Module
Quản lý progress bar và animation
"""
import tkinter as tk
import threading
import time


class ProgressManager:
    """Quản lý progress bar cho Bag Mapping Interface"""
    
    def __init__(self, root, progress_bar, upload_stat_label, translator, config):
        """
        Initialize progress manager
        
        Args:
            root: Tkinter root window
            progress_bar: Tkinter Progressbar widget
            upload_stat_label: Tkinter Label để hiển thị status
            translator: Translator instance
            config: BagMappingConfig instance
        """
        self.root = root
        self.progress = progress_bar
        self.upload_stat = upload_stat_label
        self.translator = translator
        self.config = config
        
        # Queue system cho smooth animation
        self.progress_queue = None  # (target_value, current_value)
        self.progress_thread_running = True
        
        # Start progress update thread
        threading.Thread(target=self._progress_update_worker, daemon=True).start()
    
    def set_progress(self, value):
        """Set progress bar với smooth animation"""
        try:
            current_value = self.progress['value']
            # Clamp value
            value = max(0, min(100, value))
            # Set vào queue để animate
            self.progress_queue = (value, current_value)
        except:
            # Fallback: set trực tiếp
            try:
                self.progress['value'] = value
            except:
                pass
    
    def _progress_update_worker(self):
        """Worker thread để animate progress bar mượt hơn"""
        while self.progress_thread_running:
            try:
                if self.progress_queue is not None:
                    target_value, current_value = self.progress_queue
                    
                    # Tính toán giá trị mới với animation
                    if abs(target_value - current_value) < self.config.progress_animation_speed:
                        # Đã đạt target
                        new_value = target_value
                        self.progress_queue = None
                    else:
                        # Di chuyển về phía target
                        if target_value > current_value:
                            new_value = min(current_value + self.config.progress_animation_speed, target_value)
                        else:
                            new_value = max(current_value - self.config.progress_animation_speed, target_value)
                        self.progress_queue = (target_value, new_value)
                    
                    # Update UI trong main thread
                    self.root.after(0, lambda v=new_value: self._update_progress_bar(v))
                
                # Sleep ngắn để animation mượt
                time.sleep(0.03)  # ~33 FPS cho animation
            except Exception as e:
                time.sleep(0.1)
    
    def _update_progress_bar(self, value):
        """Update progress bar với giá trị mới"""
        try:
            self.progress['value'] = value
            # Update status text
            if value >= 100:
                self.upload_stat.config(text=self.translator.get('label.upload_done', '100% - Done'))
            elif value > 0:
                self.upload_stat.config(text=self.translator.get('label.uploading', 'Uploading: {val}%').replace('{val}', str(int(value))))
        except:
            pass
    
    def stop(self):
        """Dừng progress thread"""
        self.progress_thread_running = False
