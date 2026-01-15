import tkinter as tk
from tkinter import messagebox
import uuid
import requests # Cài đặt bằng lệnh: pip install requests
from contants.API import VEHICLE_ENDPOINT, API_TIMEOUT, HEADERS, BACKEND_HOST
from languages.translate_engine import Translator


class RegistrationInterface:
    def __init__(self, root):
        self.root = root
        self.root.geometry("500x500")
        
        # Translator for multi-language support
        self.translator = Translator('en')
        self.current_lang = 'en'
        
        # Fetch available vehicle categories from backend
        self.available_categories = []
        self.fetch_categories()
        
        # Setup language button FIRST (before main container)
        self.setup_language_button()
        
        # --- Main Container ---
        self.main_container = tk.Frame(root)
        self.main_container.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        # --- Workspace ---
        self.workspace = tk.Frame(self.main_container)
        self.workspace.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=10, pady=10)

        # Căn giữa Form
        self.form_container = tk.LabelFrame(self.workspace, text=self.translator.get("label.registration_form"), padx=20, pady=20)
        self.form_container.place(relx=0.5, rely=0.45, anchor=tk.CENTER)

        self.setup_registration_form()

        # Version info
        tk.Label(self.workspace, text="", 
                 fg="grey", font=("Arial", 8)).pack(side=tk.BOTTOM, pady=10)
        
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
        """Toggle between English and Japanese"""
        if self.current_lang == 'en':
            self.change_language('jp')
        else:
            self.change_language('en')
    
    def change_language(self, lang_code):
        """Change language and update UI"""
        self.current_lang = lang_code
        self.translator.switch_language(lang_code)
        
        lang_names = {'en': 'English', 'jp': '日本語'}
        self.lang_button.config(text=f"🌐 {lang_names.get(lang_code, 'English')}")
        
        self.update_ui_texts()
    
    def update_ui_texts(self):
        """Update all UI texts based on current language"""
        self.root.title(self.translator.get("label.registration_system"))
        
        # Update form container title
        if hasattr(self, 'form_container'):
            self.form_container.config(text=self.translator.get("label.registration_form"))
        
        # Update labels
        if hasattr(self, 'label_vehicle_id'):
            self.label_vehicle_id.config(text=self.translator.get("label.vehicle_id_system_id"))
        if hasattr(self, 'label_display_name'):
            self.label_display_name.config(text=self.translator.get("label.display_name"))
        if hasattr(self, 'label_vehicle_category'):
            self.label_vehicle_category.config(text=self.translator.get("label.vehicle_category"))
        if hasattr(self, 'label_type_category'):
            self.label_type_category.config(text=self.translator.get("label.type_category"))
        if hasattr(self, 'label_description'):
            self.label_description.config(text=self.translator.get("label.description_and_notes"))
        
        # Update vehicle category dropdown with new translations
        if hasattr(self, 'opt_vehicle_category') and self.available_categories:
            current_value = self.var_vehicle_category.get()
            # Find the category value for current selection
            current_category_value = None
            if current_value and hasattr(self, 'category_value_map'):
                current_category_value = self.category_value_map.get(current_value)
            
            # Rebuild options with new translations
            category_options = [""]
            self.category_value_map = {}
            for cat in self.available_categories:
                translated_label = self.translator.get(f"category.{cat}", cat)
                category_options.append(translated_label)
                self.category_value_map[translated_label] = cat
            
            # Recreate dropdown menu
            menu = self.opt_vehicle_category["menu"]
            menu.delete(0, "end")
            for option in category_options:
                menu.add_command(label=option, command=lambda val=option: self.var_vehicle_category.set(val))
            
            # Restore selection if possible
            if current_category_value:
                new_label = self.translator.get(f"category.{current_category_value}", current_category_value)
                if new_label in category_options:
                    self.var_vehicle_category.set(new_label)
                else:
                    self.var_vehicle_category.set("")
            else:
                self.var_vehicle_category.set("")
        
        # Update buttons
        if hasattr(self, 'btn_cancel'):
            self.btn_cancel.config(text=self.translator.get("button.cancel"))
        if hasattr(self, 'btn_register'):
            self.btn_register.config(text=self.translator.get("button.regiter"))
        
    def get_mac(self):
        mac = ':'.join(['{:02x}'.format((uuid.getnode() >> ele) & 0xff)
                        for ele in range(0, 8*6, 8)][::-1])
        return mac

    def fetch_categories(self):
        """Fetch available vehicle categories from backend API."""
        try:
            categories_url = f"{BACKEND_HOST}/api/v1/vehicles/categories"
            response = requests.get(categories_url, headers=HEADERS, timeout=API_TIMEOUT)
            if response.status_code == 200:
                data = response.json()
                self.available_categories = data.get("categories", [])
                print(f"Fetched {len(self.available_categories)} vehicle categories from backend")
            else:
                print(f"Failed to fetch categories: HTTP {response.status_code}")
                self.available_categories = []
        except requests.exceptions.RequestException as e:
            print(f"Error fetching categories: {e}")
            self.available_categories = []

    def setup_registration_form(self):
        # Cấu hình grid
        self.form_container.columnconfigure(0, weight=1)

        # 1. Vehicle ID
        self.label_vehicle_id = tk.Label(self.form_container, text=self.translator.get("label.vehicle_id_system_id"))
        self.label_vehicle_id.grid(row=0, column=0, sticky="w", pady=(0, 2))
        self.ent_vehicle_id = tk.Entry(self.form_container, width=40)
        mac = self.get_mac()
        vehicle_id = mac.lower().replace(':', '')[:12]
        self.ent_vehicle_id.insert(0, vehicle_id)
        self.ent_vehicle_id.grid(row=1, column=0, sticky="ew", pady=(0, 10))

        # 2. Display Name
        self.label_display_name = tk.Label(self.form_container, text=self.translator.get("label.display_name"))
        self.label_display_name.grid(row=2, column=0, sticky="w", pady=(0, 2))
        self.ent_display_name = tk.Entry(self.form_container)
        self.ent_display_name.grid(row=3, column=0, sticky="ew", pady=(0, 10))

        # 3. Vehicle Category (from backend API)
        self.label_vehicle_category = tk.Label(self.form_container, text=self.translator.get("label.vehicle_category"))
        self.label_vehicle_category.grid(row=4, column=0, sticky="w", pady=(0, 2))
        self.var_vehicle_category = tk.StringVar(value="")
        # Build options for dropdown with translations
        category_options = [""]  # Empty option first
        if self.available_categories:
            for cat in self.available_categories:
                # Get translated label, fallback to category value if translation not found
                translated_label = self.translator.get(f"category.{cat}", cat)
                category_options.append(translated_label)
        else:
            # Fallback if categories not loaded
            category_options = [""]
        # Store mapping from translated label to category value
        self.category_value_map = {}
        if self.available_categories:
            for cat in self.available_categories:
                translated_label = self.translator.get(f"category.{cat}", cat)
                self.category_value_map[translated_label] = cat
        self.opt_vehicle_category = tk.OptionMenu(self.form_container, self.var_vehicle_category, *category_options)
        self.opt_vehicle_category.grid(row=5, column=0, sticky="ew", pady=(0, 10))

        # 4. Vehicle Type (keep existing Scanner/Worker dropdown)
        self.label_type_category = tk.Label(self.form_container, text=self.translator.get("label.type_category"))
        self.label_type_category.grid(row=6, column=0, sticky="w", pady=(0, 2))
        self.var_category = tk.StringVar(value="Scanner")
        opt_category = tk.OptionMenu(self.form_container, self.var_category, "Scanner", "Worker")
        opt_category.grid(row=7, column=0, sticky="ew", pady=(0, 10))

        # 5. Description
        self.label_description = tk.Label(self.form_container, text=self.translator.get("label.description_and_notes"))
        self.label_description.grid(row=8, column=0, sticky="w", pady=(0, 2))
        self.txt_desc = tk.Text(self.form_container, height=4, width=40, font=("Arial", 10))
        self.txt_desc.grid(row=9, column=0, sticky="ew", pady=(0, 15))

        # 6. Buttons
        btn_frame = tk.Frame(self.form_container)
        btn_frame.grid(row=10, column=0, sticky="e")

        self.btn_cancel = tk.Button(btn_frame, text=self.translator.get("button.cancel"), width=10, command=self.root.quit)
        self.btn_cancel.pack(side=tk.LEFT, padx=5)

        # Giữ nguyên format nút bấm cũ
        self.btn_register = tk.Button(btn_frame, text=self.translator.get("button.regiter"), width=15, 
                                 command=self.submit_form)
        self.btn_register.pack(side=tk.LEFT)

    def submit_form(self):
        # Lấy dữ liệu từ form
        v_id = self.ent_vehicle_id.get().strip()
        name = self.ent_display_name.get().strip()
        vehicle_type = self.var_category.get()
        vehicle_category_label = self.var_vehicle_category.get()
        desc = self.txt_desc.get("1.0", tk.END).strip()

        if not name:
            messagebox.showwarning(
                self.translator.get("dialog.warning"),
                self.translator.get("message.please_enter_display_name")
            )
            return
       
        payload = {
            "vehicle_id": v_id,
            "name": name,
            "description": desc,
            "vehicle_type": vehicle_type,
            "metadata": {
                "additionalProp1": {}
            }
        }
        
        # Convert translated label back to category value if selected
        if vehicle_category_label and hasattr(self, 'category_value_map'):
            vehicle_category = self.category_value_map.get(vehicle_category_label, vehicle_category_label)
            if vehicle_category:
                payload["vehicle_category"] = vehicle_category

        print("Submitting registration with payload:", payload)
        print("Submitting registration with url VEHICLE_ENDPOINT:", VEHICLE_ENDPOINT)

        try:
            response = requests.post(VEHICLE_ENDPOINT, json=payload, headers=HEADERS, timeout=5)
            
            if response.status_code in [200, 201]:
                # 1. Hiện thông báo thành công
                success_message = self.translator.get("message.registered_successfully").replace("{name}", name)
                messagebox.showinfo(self.translator.get("dialog.success"), success_message)
                
                # 2. Đóng cửa sổ hiện tại để script .sh chạy tiếp
                self.root.destroy() 
            else:
                error_message = self.translator.get("message.registration_error").replace("{status_code}", str(response.status_code)).replace("{response_text}", response.text)
                messagebox.showerror(self.translator.get("dialog.error"), error_message)
        
        except requests.exceptions.RequestException as e:
            error_message = self.translator.get("message.connection_error").replace("{endpoint}", VEHICLE_ENDPOINT).replace("{error}", str(e))
            messagebox.showerror(self.translator.get("dialog.connection_error"), error_message)

if __name__ == "__main__":
    root = tk.Tk()
    app = RegistrationInterface(root)
    root.mainloop()