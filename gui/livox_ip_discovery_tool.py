#!/usr/bin/env python3
"""
Livox IP Discovery Tool
Tool để tự động tìm kiếm hoặc nhập thủ công IP của thiết bị Livox MID360
và cập nhật vào các file config
"""

import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext
import threading
from pathlib import Path
import subprocess
import sys
import os

# Add current directory to path for imports
current_dir = Path(__file__).parent
if str(current_dir) not in sys.path:
    sys.path.insert(0, str(current_dir))

# Import Translator với error handling
try:
    sys.path.insert(0, str(Path(__file__).parent.parent / "languages"))
    from translate_engine import Translator
except (ImportError, SystemError, OSError) as e:
    print(f"Error: Cannot import Translator: {e}")
    # Fallback Translator class
    class Translator:
        def __init__(self, lang='en'):
            self.lang = lang
        def get(self, key, default=None):
            return default or key
        def switch_language(self, lang):
            self.lang = lang
    Translator = Translator

from livox_network_scanner import LivoxNetworkScanner
from livox_config_updater import LivoxConfigUpdater


class LivoxIPDiscoveryTool:
    """Main GUI application for Livox IP discovery and configuration"""
    
    def __init__(self, root):
        self.root = root
        
        # Initialize translator
        self.translator = Translator('en')
        self.current_lang = 'en'
        
        self.root.title(self.translator.get('title.livox_ip_discovery', 'Livox MID360 IP Discovery Tool'))
        self.root.geometry("900x700")
        
        # Initialize modules
        self.scanner = LivoxNetworkScanner(translator=self.translator)
        self.config_updater = LivoxConfigUpdater(translator=self.translator)
        
        # Variables
        self.selected_device = None
        self.scan_results = []
        
        # Setup language button FIRST (before main container)
        self.setup_language_button()
        
        # Create UI
        self.create_widgets()
        
        # Load current IP from config
        self.load_current_ip()
        
        # Update UI texts after all components are set up
        self.update_ui_texts()
    
    def setup_language_button(self):
        """Setup language toggle button"""
        lang_frame = tk.Frame(self.root)
        lang_frame.pack(side=tk.TOP, fill=tk.X, padx=5, pady=5)
        
        lang_names = {'en': 'English', 'jp': '日本語'}
        self.lang_button = tk.Button(
            lang_frame,
            text=f"🌐 {lang_names.get(self.current_lang, 'English')}",
            font=("Arial", 9, "bold"),
            fg="#0066cc",
            bg="#e8e8e8",
            activebackground="#d0d0d0",
            activeforeground="#0066cc",
            relief=tk.RAISED,
            bd=1,
            padx=12,
            pady=6,
            cursor="hand2",
            command=self.toggle_language
        )
        self.lang_button.pack(side=tk.RIGHT)
    
    def toggle_language(self):
        """Toggle between languages"""
        if self.current_lang == 'en':
            self.change_language('jp')
        else:
            self.change_language('en')
    
    def change_language(self, lang_code):
        """Change language"""
        self.current_lang = lang_code
        self.translator.switch_language(lang_code)
        
        lang_names = {'en': 'English', 'jp': '日本語'}
        self.lang_button.config(text=f"🌐 {lang_names.get(lang_code, 'English')}")
        
        self.update_ui_texts()
    
    def update_ui_texts(self):
        """Update all UI texts with current language"""
        self.root.title(self.translator.get('title.livox_ip_discovery', 'Livox MID360 IP Discovery Tool'))
        
        # Update notebook tabs
        if hasattr(self, 'notebook'):
            self.notebook.tab(0, text=self.translator.get('label.auto_discovery', '🔍 Auto Discovery'))
            self.notebook.tab(1, text=self.translator.get('label.manual_input', '✏️ Manual Input'))
        
        # Update auto discovery tab
        if hasattr(self, 'scan_button'):
            self.scan_button.config(text=self.translator.get('button.scan_network', 'Scan Network'))
        if hasattr(self, 'stop_scan_button'):
            self.stop_scan_button.config(text=self.translator.get('button.stop_scan', 'Stop Scan'))
        if hasattr(self, 'update_config_button'):
            self.update_config_button.config(text=self.translator.get('button.select_and_update', 'Select and Update Config'))
        if hasattr(self, 'build_after_update_var'):
            # Checkbox text is set in create_widgets, need to update it
            pass
        
        # Update manual input tab
        if hasattr(self, 'test_button'):
            self.test_button.config(text=self.translator.get('button.test_connection', 'Test Connection'))
        if hasattr(self, 'manual_update_button'):
            self.manual_update_button.config(text=self.translator.get('button.update_config', 'Update Config'))
        
        # Update status bar
        if hasattr(self, 'status_bar'):
            self.status_bar.config(text=self.translator.get('label.ready', 'Ready'))
    
    def create_widgets(self):
        """Create GUI widgets"""
        # Create notebook for tabs
        self.notebook = ttk.Notebook(self.root)
        self.notebook.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Tab 1: Auto Discovery
        self.auto_frame = ttk.Frame(self.notebook, padding="10")
        self.notebook.add(self.auto_frame, text=self.translator.get('label.auto_discovery', '🔍 Auto Discovery'))
        self.create_auto_discovery_tab()
        
        # Tab 2: Manual Input
        self.manual_frame = ttk.Frame(self.notebook, padding="10")
        self.notebook.add(self.manual_frame, text=self.translator.get('label.manual_input', '✏️ Manual Input'))
        self.create_manual_input_tab()
        
        # Status bar
        self.status_bar = ttk.Label(self.root, text=self.translator.get('label.ready', 'Ready'), relief=tk.SUNKEN, anchor=tk.W)
        self.status_bar.pack(side=tk.BOTTOM, fill=tk.X)
    
    def create_auto_discovery_tab(self):
        """Create auto discovery tab"""
        # Control frame
        control_frame = ttk.Frame(self.auto_frame)
        control_frame.pack(fill=tk.X, pady=(0, 10))
        
        # Scan button
        self.scan_button = ttk.Button(
            control_frame,
            text=self.translator.get('button.scan_network', 'Scan Network'),
            command=self.start_scan,
            width=20
        )
        self.scan_button.pack(side=tk.LEFT, padx=5)
        
        # Stop scan button
        self.stop_scan_button = ttk.Button(
            control_frame,
            text=self.translator.get('button.stop_scan', 'Stop Scan'),
            command=self.stop_scan,
            state=tk.DISABLED,
            width=20
        )
        self.stop_scan_button.pack(side=tk.LEFT, padx=5)
        
        # Current IP label
        self.current_ip_label = ttk.Label(
            control_frame,
            text=self.translator.get('label.current_ip', 'Current IP: --').replace('{ip}', '--'),
            font=('Arial', 10)
        )
        self.current_ip_label.pack(side=tk.LEFT, padx=20)
        
        # Results frame
        results_frame = ttk.LabelFrame(self.auto_frame, text=self.translator.get('label.scan_results', 'Scan Results'), padding="10")
        results_frame.pack(fill=tk.BOTH, expand=True, pady=(0, 10))
        
        # Treeview for results
        columns = ('IP', 'Status', 'Confidence', 'Connectivity', 'Ports')
        self.results_tree = ttk.Treeview(results_frame, columns=columns, show='headings', height=10)
        
        # Configure columns
        self.results_tree.heading('IP', text='IP Address')
        self.results_tree.heading('Status', text=self.translator.get('label.connection', 'Connection'))
        self.results_tree.heading('Confidence', text=self.translator.get('label.confidence_percent', 'Confidence %'))
        self.results_tree.heading('Connectivity', text=self.translator.get('label.connectivity', 'Connectivity'))
        self.results_tree.heading('Ports', text=self.translator.get('label.open_ports', 'Open Ports'))
        
        self.results_tree.column('IP', width=150)
        self.results_tree.column('Status', width=120)
        self.results_tree.column('Confidence', width=100)
        self.results_tree.column('Connectivity', width=100)
        self.results_tree.column('Ports', width=200)
        
        # Scrollbar for treeview
        scrollbar_tree = ttk.Scrollbar(results_frame, orient=tk.VERTICAL, command=self.results_tree.yview)
        self.results_tree.configure(yscrollcommand=scrollbar_tree.set)
        
        self.results_tree.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar_tree.pack(side=tk.RIGHT, fill=tk.Y)
        
        # Bind selection event
        self.results_tree.bind('<<TreeviewSelect>>', self.on_device_select)
        
        # Action frame
        action_frame = ttk.Frame(self.auto_frame)
        action_frame.pack(fill=tk.X, pady=(0, 10))
        
        # Checkbox for build after update
        self.build_after_update_var = tk.BooleanVar(value=True)
        build_checkbox = ttk.Checkbutton(
            action_frame,
            text=self.translator.get('label.build_after_update', 'Rebuild driver after update'),
            variable=self.build_after_update_var
        )
        build_checkbox.pack(side=tk.LEFT, padx=5)
        
        # Update config button
        self.update_config_button = ttk.Button(
            action_frame,
            text=self.translator.get('button.select_and_update', 'Select and Update Config'),
            command=self.update_config_from_selection,
            state=tk.DISABLED,
            width=30
        )
        self.update_config_button.pack(side=tk.LEFT, padx=5)
        
        # Log frame
        log_frame = ttk.LabelFrame(self.auto_frame, text=self.translator.get('label.log', 'Log'), padding="10")
        log_frame.pack(fill=tk.BOTH, expand=True)
        
        self.log_text = scrolledtext.ScrolledText(log_frame, height=8, wrap=tk.WORD, state=tk.DISABLED)
        self.log_text.pack(fill=tk.BOTH, expand=True)
    
    def create_manual_input_tab(self):
        """Create manual input tab"""
        # IP input frame
        input_frame = ttk.LabelFrame(self.manual_frame, text=self.translator.get('label.enter_ip', 'Enter IP'), padding="10")
        input_frame.pack(fill=tk.X, pady=(0, 10))
        
        ttk.Label(input_frame, text=self.translator.get('label.ip_address', 'IP Address:')).pack(side=tk.LEFT, padx=5)
        
        self.ip_entry = ttk.Entry(input_frame, width=20, font=('Arial', 12))
        self.ip_entry.pack(side=tk.LEFT, padx=5)
        self.ip_entry.insert(0, "192.168.1.")
        
        # Current IP from config
        self.manual_current_ip_label = ttk.Label(
            input_frame,
            text=self.translator.get('label.current_ip', 'Current IP: --').replace('{ip}', '--'),
            font=('Arial', 10)
        )
        self.manual_current_ip_label.pack(side=tk.LEFT, padx=20)
        
        # Buttons frame
        buttons_frame = ttk.Frame(self.manual_frame)
        buttons_frame.pack(fill=tk.X, pady=(0, 10))
        
        # Checkbox for build after update
        self.manual_build_after_update_var = tk.BooleanVar(value=True)
        build_checkbox = ttk.Checkbutton(
            buttons_frame,
            text=self.translator.get('label.build_after_update', 'Rebuild driver after update'),
            variable=self.manual_build_after_update_var
        )
        build_checkbox.pack(side=tk.LEFT, padx=5)
        
        # Test connection button
        self.test_button = ttk.Button(
            buttons_frame,
            text=self.translator.get('button.test_connection', 'Test Connection'),
            command=self.test_connection,
            width=20
        )
        self.test_button.pack(side=tk.LEFT, padx=5)
        
        # Update config button
        self.manual_update_button = ttk.Button(
            buttons_frame,
            text=self.translator.get('button.update_config', 'Update Config'),
            command=self.update_config_manual,
            width=20
        )
        self.manual_update_button.pack(side=tk.LEFT, padx=5)
        
        # Connection status frame
        status_frame = ttk.LabelFrame(self.manual_frame, text=self.translator.get('label.connection_status', 'Connection Status'), padding="10")
        status_frame.pack(fill=tk.X, pady=(0, 10))
        
        self.connection_status_label = ttk.Label(
            status_frame,
            text=self.translator.get('label.not_tested', 'Not tested'),
            font=('Arial', 10)
        )
        self.connection_status_label.pack(anchor=tk.W)
        
        # Log frame
        log_frame = ttk.LabelFrame(self.manual_frame, text=self.translator.get('label.log', 'Log'), padding="10")
        log_frame.pack(fill=tk.BOTH, expand=True)
        
        self.manual_log_text = scrolledtext.ScrolledText(log_frame, height=15, wrap=tk.WORD, state=tk.DISABLED)
        self.manual_log_text.pack(fill=tk.BOTH, expand=True)
    
    def log(self, message, tab='auto'):
        """Add message to log"""
        log_widget = self.log_text if tab == 'auto' else self.manual_log_text
        
        def update_log():
            log_widget.config(state=tk.NORMAL)
            log_widget.insert(tk.END, f"{message}\n")
            log_widget.see(tk.END)
            log_widget.config(state=tk.DISABLED)
        
        if threading.current_thread() is threading.main_thread():
            update_log()
        else:
            self.root.after(0, update_log)
    
    def update_status(self, message):
        """Update status bar"""
        def update():
            self.status_bar.config(text=message)
        
        if threading.current_thread() is threading.main_thread():
            update()
        else:
            self.root.after(0, update)
    
    def load_current_ip(self):
        """Load current IP from config files"""
        try:
            current_ips = self.config_updater.get_current_ip()
            drive_ws_ip = current_ips.get('drive_ws', '--')
            sdk2_ip = current_ips.get('sdk2', '--')
            
            # Display current IP
            if drive_ws_ip == sdk2_ip and drive_ws_ip != '--':
                ip_text = self.translator.get('label.current_ip', 'Current IP: {ip}').replace('{ip}', drive_ws_ip)
            else:
                ip_text = self.translator.get('label.current_ip', 'Current IP: {ip}').replace('{ip}', f"drive_ws={drive_ws_ip}, sdk2={sdk2_ip}")
            
            self.current_ip_label.config(text=ip_text)
            self.manual_current_ip_label.config(text=ip_text)
            
            # Set default in manual entry
            if drive_ws_ip and drive_ws_ip != '--':
                self.ip_entry.delete(0, tk.END)
                self.ip_entry.insert(0, drive_ws_ip)
        except Exception as e:
            self.log(f"Error loading current IP: {e}")
    
    def start_scan(self):
        """Start network scan"""
        if self.scanner.is_scanning:
            messagebox.showwarning(
                self.translator.get('dialog.warning', 'Warning'),
                self.translator.get('message.scanning_network_wait', 'Scanning network, please wait...')
            )
            return
        
        # Clear previous results
        for item in self.results_tree.get_children():
            self.results_tree.delete(item)
        self.scan_results = []
        self.selected_device = None
        self.update_config_button.config(state=tk.DISABLED)
        
        # Update UI
        self.scan_button.config(state=tk.DISABLED)
        self.stop_scan_button.config(state=tk.NORMAL)
        self.update_status(self.translator.get('message.scanning_network', 'Scanning network...'))
        
        # Start scan in thread
        def scan_thread():
            self.log(self.translator.get('message.starting_network_scan', 'Starting network scan...'))
            
            def scan_callback(message):
                self.log(message)
            
            self.scanner.scan_current_network(scan_callback)
            
            # Wait for scan to complete
            while self.scanner.is_scanning:
                threading.Event().wait(0.5)
            
            # Update results
            self.root.after(0, self.update_scan_results)
        
        threading.Thread(target=scan_thread, daemon=True).start()
    
    def stop_scan(self):
        """Stop network scan"""
        self.scanner.stop_scanning()
        self.scan_button.config(state=tk.NORMAL)
        self.stop_scan_button.config(state=tk.DISABLED)
        self.update_status(self.translator.get('message.scan_stopped', 'Scan stopped'))
        self.log(self.translator.get('message.scan_stopped', 'Scan stopped'))
    
    def update_scan_results(self):
        """Update scan results in treeview"""
        # Clear previous results
        for item in self.results_tree.get_children():
            self.results_tree.delete(item)
        
        # Get Livox devices
        livox_devices = self.scanner.get_livox_devices()
        self.scan_results = livox_devices
        
        # Sort by connectivity score (directly connected first), then by confidence
        livox_devices_sorted = sorted(
            livox_devices,
            key=lambda x: (
                not x.get('is_directly_connected', False),
                -x.get('connectivity_score', 0),
                -x['confidence']
            )
        )
        
        # Add to treeview
        for device in livox_devices_sorted:
            connection_status = self.translator.get('label.direct_connection', '🔌 Direct') if device.get('is_directly_connected', False) else self.translator.get('label.network_connection', '🌐 Network')
            open_ports = device.get('open_ports', [])[:5]
            ports_str = ', '.join(map(str, open_ports))
            if len(device.get('open_ports', [])) > 5:
                ports_str += "..."
            
            self.results_tree.insert('', tk.END, values=(
                device['ip'],
                connection_status,
                f"{device['confidence']}%",
                f"{device.get('connectivity_score', 0)}/100",
                ports_str
            ))
        
        # Update UI
        self.scan_button.config(state=tk.NORMAL)
        self.stop_scan_button.config(state=tk.DISABLED)
        self.update_status(self.translator.get('message.found_livox_devices', 'Found {count} Livox devices').replace('{count}', str(len(livox_devices))))
        
        if len(livox_devices) > 0:
            # Auto-select first device (usually the directly connected one)
            first_item = self.results_tree.get_children()[0]
            self.results_tree.selection_set(first_item)
            self.results_tree.focus(first_item)
            self.on_device_select(None)
    
    def on_device_select(self, event):
        """Handle device selection"""
        selection = self.results_tree.selection()
        if not selection:
            self.selected_device = None
            self.update_config_button.config(state=tk.DISABLED)
            return
        
        # Get selected item
        item = selection[0]
        values = self.results_tree.item(item, 'values')
        selected_ip = values[0]
        
        # Find device in results
        self.selected_device = next(
            (d for d in self.scan_results if d['ip'] == selected_ip),
            None
        )
        
        if self.selected_device:
            self.update_config_button.config(state=tk.NORMAL)
            # Show device details in log
            self.log(self.translator.get('message.device_selected', 'Device selected: {ip}').replace('{ip}', selected_ip))
            self.log(f"  {self.translator.get('label.confidence_percent', 'Confidence %')}: {self.selected_device['confidence']}%")
            self.log(f"  {self.translator.get('label.connectivity', 'Connectivity')}: {self.selected_device.get('connectivity_score', 0)}/100")
            connection_type = self.translator.get('label.direct_connection', 'Direct') if self.selected_device.get('is_directly_connected', False) else self.translator.get('label.network_connection', 'Network')
            self.log(f"  {self.translator.get('label.connection', 'Connection')}: {connection_type}")
            if 'connectivity_reasons' in self.selected_device:
                self.log(f"  {self.translator.get('message.connectivity_reasons', 'Reasons:')}")
                for reason in self.selected_device['connectivity_reasons']:
                    self.log(f"    {reason}")
    
    def update_config_from_selection(self):
        """Update config from selected device"""
        if not self.selected_device:
            messagebox.showwarning(
                self.translator.get('dialog.warning', 'Warning'),
                self.translator.get('message.select_device_from_list', 'Please select a device from the list')
            )
            return
        
        ip = self.selected_device['ip']
        build_after = self.build_after_update_var.get()
        self.update_config(ip, 'auto', build_after)
    
    def test_connection(self):
        """Test connection to entered IP"""
        ip = self.ip_entry.get().strip()
        
        if not ip:
            messagebox.showwarning(
                self.translator.get('dialog.warning', 'Warning'),
                self.translator.get('message.enter_ip_address', 'Please enter IP address')
            )
            return
        
        # Validate IP
        is_valid, error_msg = self.config_updater.validate_ip(ip)
        if not is_valid:
            messagebox.showerror(self.translator.get('dialog.error', 'Error'), error_msg)
            return
        
        # Test in thread
        def test_thread():
            self.manual_log_text.config(state=tk.NORMAL)
            self.manual_log_text.insert(tk.END, self.translator.get('message.testing_connection_to', 'Testing connection to {ip}...').replace('{ip}', ip) + "\n")
            self.manual_log_text.see(tk.END)
            self.manual_log_text.config(state=tk.DISABLED)
            
            self.root.after(0, lambda: self.connection_status_label.config(
                text=self.translator.get('label.testing', 'Testing...'),
                foreground="orange"
            ))
            
            # Ping test
            try:
                result = subprocess.run(
                    ['ping', '-c', '3', '-W', '2', ip],
                    capture_output=True,
                    text=True,
                    timeout=10
                )
                
                if result.returncode == 0:
                    # Parse ping results
                    lines = result.stdout.split('\n')
                    for line in lines:
                        if 'time=' in line:
                            time_part = line.split('time=')[1].split()[0]
                            self.root.after(0, lambda: self.connection_status_label.config(
                                text=self.translator.get('label.connection_success', '✅ Connection successful (latency: {latency})').replace('{latency}', time_part),
                                foreground="green"
                            ))
                            self.log(self.translator.get('label.connection_success', '✅ Connection successful (latency: {latency})').replace('{latency}', time_part), 'manual')
                            break
                else:
                    self.root.after(0, lambda: self.connection_status_label.config(
                        text=self.translator.get('label.connection_failed', '❌ Cannot connect'),
                        foreground="red"
                    ))
                    self.log(self.translator.get('label.connection_failed', '❌ Cannot connect'), 'manual')
            except Exception as e:
                self.root.after(0, lambda: self.connection_status_label.config(
                    text=self.translator.get('label.error', 'Error: {error}').replace('{error}', str(e)),
                    foreground="red"
                ))
                self.log(self.translator.get('label.error', 'Error: {error}').replace('{error}', str(e)), 'manual')
        
        threading.Thread(target=test_thread, daemon=True).start()
    
    def update_config_manual(self):
        """Update config from manual input"""
        ip = self.ip_entry.get().strip()
        
        if not ip:
            messagebox.showwarning(
                self.translator.get('dialog.warning', 'Warning'),
                self.translator.get('message.enter_ip_address', 'Please enter IP address')
            )
            return
        
        build_after = self.manual_build_after_update_var.get()
        self.update_config(ip, 'manual', build_after)
    
    def update_config(self, ip, source='auto', build_after=True):
        """Update config files with new IP"""
        # Confirm
        build_text = " " + self.translator.get('message.confirm_update_ip', 'and rebuild driver') if build_after else ""
        if build_after:
            build_text = " and rebuild driver"
        else:
            build_text = ""
        confirm_msg = self.translator.get('message.confirm_update_ip', 'Are you sure you want to update IP to {ip}{build_text}?').replace('{ip}', ip).replace('{build_text}', build_text)
        if not messagebox.askyesno(self.translator.get('dialog.warning', 'Warning'), confirm_msg):
            return
        
        # Disable buttons when starting
        def disable_buttons():
            if source == 'auto':
                self.update_config_button.config(state=tk.DISABLED)
                self.scan_button.config(state=tk.DISABLED)
            else:
                self.manual_update_button.config(state=tk.DISABLED)
                self.test_button.config(state=tk.DISABLED)
        
        def enable_buttons():
            if source == 'auto':
                # Re-enable based on selection
                if self.selected_device:
                    self.update_config_button.config(state=tk.NORMAL)
                self.scan_button.config(state=tk.NORMAL)
            else:
                self.manual_update_button.config(state=tk.NORMAL)
                self.test_button.config(state=tk.NORMAL)
        
        # Disable buttons immediately
        self.root.after(0, disable_buttons)
        
        # Update in thread
        def update_thread():
            log_widget = self.log_text if source == 'auto' else self.manual_log_text
            
            def log_msg(msg):
                log_widget.config(state=tk.NORMAL)
                log_widget.insert(tk.END, f"{msg}\n")
                log_widget.see(tk.END)
                log_widget.config(state=tk.DISABLED)
            
            self.root.after(0, lambda: self.update_status(self.translator.get('message.updating_config', 'Updating config...')))
            log_msg(self.translator.get('message.updating_ip_to', 'Updating IP to {ip}...').replace('{ip}', ip))
            
            # Update config
            result = self.config_updater.update_config_files(ip)
            
            # Show results
            if result['success']:
                log_msg(self.translator.get('message.update_config_success', '✅ Config update successful!'))
                log_msg(f"  - {result['drive_ws']['message']}")
                log_msg(f"  - {result['sdk2']['message']}")
                
                # Build driver if requested
                if build_after:
                    log_msg("")
                    log_msg(self.translator.get('message.starting_build_driver', '🔨 Starting build driver...'))
                    log_msg(self.translator.get('message.build_may_take_minutes', '(This process may take a few minutes, please wait...)'))
                    self.root.after(0, lambda: self.update_status(self.translator.get('message.building_driver', 'Building driver... (please wait)')))
                    
                    build_success, build_message = self.config_updater.build_livox_driver(callback=log_msg)
                    
                    if build_success:
                        log_msg("")
                        log_msg(self.translator.get('message.build_complete', '✅ Complete! IP has been updated and driver has been rebuilt.'))
                        log_msg(self.translator.get('message.driver_ready', '🎯 You can use the driver with the new IP now!'))
                        
                        # Show success message
                        self.root.after(0, lambda: messagebox.showinfo(
                            self.translator.get('message.update_success', '✅ Success!'),
                            self.translator.get('message.update_and_build_success', 'IP updated to {ip} and driver built successfully!\n\nDriver is ready to use with the new IP.').replace('{ip}', ip)
                        ))
                        self.root.after(0, lambda: self.update_status(self.translator.get('message.update_and_build_success_status', '✅ Update and build successful!')))
                    else:
                        log_msg("")
                        log_msg(self.translator.get('message.build_driver_failed', '⚠️ IP update successful but build failed: {build_message}').replace('{build_message}', build_message))
                        self.root.after(0, lambda: messagebox.showwarning(
                            self.translator.get('message.update_success_build_failed', '⚠️ Warning'),
                            self.translator.get('message.update_success_build_failed_msg', 'IP updated to {ip} successfully!\n\nBut driver build failed:\n{build_message}\n\nYou can try building manually.').replace('{ip}', ip).replace('{build_message}', build_message)
                        ))
                        self.root.after(0, lambda: self.update_status(self.translator.get('message.update_success_build_failed_status', 'Update successful, build failed')))
                    
                    # Re-enable buttons after build
                    self.root.after(0, enable_buttons)
                else:
                    self.root.after(0, lambda: messagebox.showinfo(
                        self.translator.get('dialog.success', 'Success'),
                        self.translator.get('message.update_success_no_build', 'IP updated to {ip} successfully!').replace('{ip}', ip)
                    ))
                    self.root.after(0, lambda: self.update_status(self.translator.get('message.update_success_status', '✅ Update successful')))
                    self.root.after(0, enable_buttons)
                
                self.root.after(0, self.load_current_ip)
            else:
                log_msg(self.translator.get('message.update_config_failed', '❌ Config update failed!'))
                log_msg(f"  - {result['message']}")
                if result['drive_ws']['message']:
                    log_msg(f"  - drive_ws: {result['drive_ws']['message']}")
                if result['sdk2']['message']:
                    log_msg(f"  - sdk2: {result['sdk2']['message']}")
                
                self.root.after(0, lambda: messagebox.showerror(
                    self.translator.get('message.update_failed', '❌ Error'),
                    self.translator.get('message.update_failed_msg', 'Cannot update config:\n{message}').replace('{message}', result['message'])
                ))
                self.root.after(0, lambda: self.update_status(self.translator.get('message.update_failed_status', '❌ Update failed')))
                # Re-enable buttons on error
                self.root.after(0, enable_buttons)
        
        threading.Thread(target=update_thread, daemon=True).start()


def main():
    """Main entry point"""
    root = tk.Tk()
    app = LivoxIPDiscoveryTool(root)
    root.mainloop()


if __name__ == "__main__":
    main()
