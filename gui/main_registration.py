import tkinter as tk
from tkinter import messagebox

class RegistrationInterface:
    def __init__(self, root):
        self.root = root
        self.root.title("Registration System")
        self.root.geometry("700x600")

        # --- Top Menu Bar ---
        # self.menu_bar = tk.Label(root, text="Registration", 
        #                          anchor="w", padx=15, relief=tk.RAISED, bd=1)
        # self.menu_bar.pack(side=tk.TOP, fill=tk.X)

        # --- Main Container ---
        self.main_container = tk.Frame(root)
        self.main_container.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        # --- Sidebar ---
        # self.sidebar = tk.Frame(self.main_container, width=280, relief=tk.SUNKEN, bd=1)
        # self.sidebar.pack(side=tk.LEFT, fill=tk.Y, padx=5, pady=5)
        # self.sidebar.pack_propagate(False)

        # System Status Card trong Sidebar
        # status_frame = tk.LabelFrame(self.sidebar, text="SYSTEM STATUS", padx=10, pady=10)
        # status_frame.pack(fill=tk.X, pady=5, padx=5)
        # tk.Label(status_frame, text="● Backend: Connected").pack(anchor="w")

        # --- Workspace (Vùng chứa Form đăng ký) ---
        self.workspace = tk.Frame(self.main_container)
        self.workspace.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=10, pady=10)

        # Căn giữa Form bằng cách đặt vào một frame phụ
        self.form_container = tk.LabelFrame(self.workspace, text="VEHICLE REGISTRATION FORM", padx=20, pady=20)
        self.form_container.place(relx=0.5, rely=0.4, anchor=tk.CENTER)

        self.setup_registration_form()

        # Version info ở dưới cùng Workspace
        tk.Label(self.workspace, text="Livox Data Recorder Version 2.4.1-Stable", 
                 fg="grey", font=("Arial", 8)).pack(side=tk.BOTTOM, pady=10)

    def setup_registration_form(self):
        # Grid cấu hình: Cột 0 và Cột 1
        self.form_container.columnconfigure(0, weight=1)
        self.form_container.columnconfigure(1, weight=1)

        # Row 1: Device ID và Vehicle ID
        tk.Label(self.form_container, text="Device ID (Hardware ID)").grid(row=0, column=0, sticky="w", pady=(0, 2))
        tk.Label(self.form_container, text="Vehicle ID (System ID)").grid(row=0, column=1, sticky="w", pady=(0, 2), padx=(10, 0))
        
        ent_device_id = tk.Entry(self.form_container)
        ent_device_id.insert(0, "cc28aa1238c5")
        ent_device_id.grid(row=1, column=0, sticky="ew", pady=(0, 10))

        ent_vehicle_id = tk.Entry(self.form_container)
        ent_vehicle_id.insert(0, "cc28aa1238c5")
        ent_vehicle_id.grid(row=1, column=1, sticky="ew", pady=(0, 10), padx=(10, 0))

        # Row 2: Display Name
        tk.Label(self.form_container, text="Display Name").grid(row=2, column=0, columnspan=2, sticky="w", pady=(0, 2))
        ent_display_name = tk.Entry(self.form_container)
        ent_display_name.grid(row=3, column=0, columnspan=2, sticky="ew", pady=(0, 10))

        # Row 3: Category và Type ID
        tk.Label(self.form_container, text="Type Category").grid(row=4, column=0, sticky="w", pady=(0, 2))
        tk.Label(self.form_container, text="Vehicle Type ID").grid(row=4, column=1, sticky="w", pady=(0, 2), padx=(10, 0))

        var_category = tk.StringVar(value="Vehicle / Car")
        opt_category = tk.OptionMenu(self.form_container, var_category, "UAV / Drone", "UGV / Robot", "Vehicle / Car", "Handheld Device")
        opt_category.grid(row=5, column=0, sticky="ew", pady=(0, 10))

        var_type_id = tk.StringVar(value="Model_Livox_V1")
        opt_type_id = tk.OptionMenu(self.form_container, var_type_id, "Model_Livox_V1", "Model_Livox_H2")
        opt_type_id.grid(row=5, column=1, sticky="ew", pady=(0, 10), padx=(10, 0))

        # Row 4: License Plate
        tk.Label(self.form_container, text="License Plate / Serial Number").grid(row=6, column=0, columnspan=2, sticky="w", pady=(0, 2))
        ent_license = tk.Entry(self.form_container)
        ent_license.grid(row=7, column=0, columnspan=2, sticky="ew", pady=(0, 10))

        # Row 5: Description
        tk.Label(self.form_container, text="Description & Notes").grid(row=8, column=0, columnspan=2, sticky="w", pady=(0, 2))
        txt_desc = tk.Text(self.form_container, height=4, font=("Arial", 10))
        txt_desc.grid(row=9, column=0, columnspan=2, sticky="ew", pady=(0, 15))

        # Row 6: Buttons
        btn_frame = tk.Frame(self.form_container)
        btn_frame.grid(row=10, column=0, columnspan=2, sticky="e")

        btn_cancel = tk.Button(btn_frame, text="Cancel", width=10, command=self.root.quit)
        btn_cancel.pack(side=tk.LEFT, padx=5)

        btn_register = tk.Button(btn_frame, text="REGISTER DEVICE", width=20, 
                                 command=lambda: self.submit_form(ent_display_name.get()))
        btn_register.pack(side=tk.LEFT)

    def submit_form(self, name):
        if not name:
            messagebox.showwarning("Warning", "Please enter a Display Name")
        else:
            messagebox.showinfo("Success", f"Device '{name}' registered successfully!")

if __name__ == "__main__":
    root = tk.Tk()
    app = RegistrationInterface(root)
    root.mainloop()