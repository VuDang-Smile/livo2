"""
Language Module
Quản lý đa ngôn ngữ cho Bag Mapping Interface
"""
import tkinter as tk


class LanguageManager:
    """Quản lý ngôn ngữ cho Bag Mapping Interface"""
    
    def __init__(self, root, translator, config, ui_updater=None):
        """
        Initialize language manager
        
        Args:
            root: Tkinter root window
            translator: Translator instance
            config: BagMappingConfig instance
            ui_updater: Function để update UI texts (optional)
        """
        self.root = root
        self.translator = translator
        self.config = config
        self.ui_updater = ui_updater
        self.current_lang = config.default_language
        self.lang_button = None
    
    def setup_language_button(self):
        """Setup language toggle button"""
        lang_frame = tk.Frame(self.root)
        lang_frame.pack(side=tk.TOP, fill=tk.X, padx=5, pady=5)
        
        lang_names = self.config.supported_languages
        self.lang_button = tk.Button(
            lang_frame,
            text=f"🌐 {lang_names.get(self.current_lang, 'English')}",
            font=self.config.font_primary,
            fg=self.config.color_primary,
            bg=self.config.color_bg_light,
            activebackground=self.config.color_bg_active,
            activeforeground=self.config.color_fg_active,
            relief=tk.RAISED,
            bd=1,
            padx=12,
            pady=6,
            cursor="hand2",
            command=self.toggle_language
        )
        self.lang_button.pack(side=tk.RIGHT)
    
    def toggle_language(self):
        """Toggle giữa các ngôn ngữ"""
        if self.current_lang == 'en':
            self.change_language('jp')
        else:
            self.change_language('en')
    
    def change_language(self, lang_code):
        """
        Thay đổi ngôn ngữ
        
        Args:
            lang_code: Mã ngôn ngữ ('en' hoặc 'jp')
        """
        self.current_lang = lang_code
        self.translator.switch_language(lang_code)
        
        lang_names = self.config.supported_languages
        if self.lang_button:
            self.lang_button.config(text=f"🌐 {lang_names.get(lang_code, 'English')}")
        
        # Update UI texts nếu có callback
        if self.ui_updater:
            self.ui_updater()
    
    def get_current_language(self):
        """Get current language code"""
        return self.current_lang
