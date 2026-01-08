import json
import os

class Translator:
    def __init__(self, lang_code):
        self.lang_code = lang_code
        self.translations = self.load_translations(lang_code)

    def load_translations(self, lang_code):
        # Lấy đường dẫn tuyệt đối đến thư mục chứa file translate_engine.py này
        base_path = os.path.dirname(os.path.abspath(__file__))
        
        # File json nằm ngay cùng thư mục (theo ảnh của bạn là languages/en.json)
        file_path = os.path.join(base_path, f'{lang_code}.json')
        
        if os.path.exists(file_path):
            with open(file_path, 'r', encoding='utf-8') as f:
                return json.load(f)
        return {}
    
    def switch_language(self, new_lang):
        self.lang_code = new_lang
        self.translations = self.load_translations(new_lang)

    def get(self, path, default=None):
        keys = path.split('.')
        data = self.translations
        try:
            for key in keys:
                data = data[key]
            return data
        except (KeyError, TypeError):
            return default or path

# Khởi tạo mặc định
translator = Translator('jp')