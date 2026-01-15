import tkinter as tk
from tkinter import messagebox
import uuid
import requests # Cài đặt bằng lệnh: pip install requests
from contants.API import VEHICLE_ENDPOINT, API_TIMEOUT, HEADERS
from languages.translate_engine import translator


class RegistrationInterface:
    def __init__(self, root):
        self.root = root
        self.root.title(translator.get("label.registration_system"))
        self.root.geometry("500x500")

        # --- Main Container ---
        self.main_container = tk.Frame(root)
        self.main_container.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        # --- Workspace ---
        self.workspace = tk.Frame(self.main_container)
        self.workspace.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=10, pady=10)

        # Căn giữa Form
        self.form_container = tk.LabelFrame(self.workspace, text=translator.get("label.registration_form"), padx=20, pady=20)
        self.form_container.place(relx=0.5, rely=0.45, anchor=tk.CENTER)

        self.setup_registration_form()

        # Version info
        tk.Label(self.workspace, text="", 
                 fg="grey", font=("Arial", 8)).pack(side=tk.BOTTOM, pady=10)
        
    def get_mac(self):
        mac = ':'.join(['{:02x}'.format((uuid.getnode() >> ele) & 0xff)
                        for ele in range(0, 8*6, 8)][::-1])
        return mac

    def setup_registration_form(self):
        # Cấu hình grid
        self.form_container.columnconfigure(0, weight=1)

        # 1. Vehicle ID
        tk.Label(self.form_container, text=translator.get("label.vehicle_id_system_id")).grid(row=0, column=0, sticky="w", pady=(0, 2))
        self.ent_vehicle_id = tk.Entry(self.form_container, width=40)
        mac = self.get_mac()
        vehicle_id = mac.lower().replace(':', '')[:12]
        self.ent_vehicle_id.insert(0, vehicle_id)
        self.ent_vehicle_id.grid(row=1, column=0, sticky="ew", pady=(0, 10))

        # 2. Display Name
        tk.Label(self.form_container, text=translator.get("label.display_name")).grid(row=2, column=0, sticky="w", pady=(0, 2))
        self.ent_display_name = tk.Entry(self.form_container)
        self.ent_display_name.grid(row=3, column=0, sticky="ew", pady=(0, 10))

        # 3. Type Category
        tk.Label(self.form_container, text=translator.get("label.type_category")).grid(row=4, column=0, sticky="w", pady=(0, 2))
        self.var_category = tk.StringVar(value="Scanner")
        opt_category = tk.OptionMenu(self.form_container, self.var_category, "Scanner", "Worker")
        opt_category.grid(row=5, column=0, sticky="ew", pady=(0, 10))

        # 4. Description
        tk.Label(self.form_container, text=translator.get("label.description_and_notes")).grid(row=6, column=0, sticky="w", pady=(0, 2))
        self.txt_desc = tk.Text(self.form_container, height=4, width=40, font=("Arial", 10))
        self.txt_desc.grid(row=7, column=0, sticky="ew", pady=(0, 15))

        # 5. Buttons
        btn_frame = tk.Frame(self.form_container)
        btn_frame.grid(row=8, column=0, sticky="e")

        btn_cancel = tk.Button(btn_frame, text=translator.get("button.cancel"), width=10, command=self.root.quit)
        btn_cancel.pack(side=tk.LEFT, padx=5)

        # Giữ nguyên format nút bấm cũ
        btn_register = tk.Button(btn_frame, text=translator.get("button.regiter"), width=15, 
                                 command=self.submit_form)
        btn_register.pack(side=tk.LEFT)

    def submit_form(self):
        # Lấy dữ liệu từ form
        v_id = self.ent_vehicle_id.get().strip()
        name = self.ent_display_name.get().strip()
        category = self.var_category.get()
        desc = self.txt_desc.get("1.0", tk.END).strip()

        if not name:
            messagebox.showwarning("Warning", "Please enter a Display Name")
            return
       
        payload = {
            "vehicle_id": v_id,
            "name": name,
            "description": desc,
            "vehicle_type": category,
            "metadata": {
                "additionalProp1": {}
            }
        }

        print("Submitting registration with payload:", payload)
        print("Submitting registration with url VEHICLE_ENDPOINT:", VEHICLE_ENDPOINT)

        try:
            response = requests.post(VEHICLE_ENDPOINT, json=payload, headers=HEADERS, timeout=5)
            
            if response.status_code in [200, 201]:
                # 1. Hiện thông báo thành công
                messagebox.showinfo("Success", f"Registered '{name}' successfully!")
                
                # 2. Đóng cửa sổ hiện tại để script .sh chạy tiếp
                self.root.destroy() 
            else:
                messagebox.showerror("Error", f"Status: {response.status_code}\n{response.text}")
        
        except requests.exceptions.RequestException as e:
            messagebox.showerror("Connection Error", f"Lỗi kết nối tới {VEHICLE_ENDPOINT}:\n{str(e)}")

if __name__ == "__main__":
    root = tk.Tk()
    app = RegistrationInterface(root)
    root.mainloop()