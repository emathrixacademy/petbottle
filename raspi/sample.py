import socket
import threading
import tkinter as tk
from tkinter import ttk
import queue
import time

# ==========================================================
# CONFIGURATION
# ==========================================================
ESP32_MAC_ADDRESS = "F4:2D:C9:71:C9:E2" 
PORT = 1 

class DualSensorGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Dual Sensor Robot Monitor")
        self.root.geometry("600x400")
        self.root.configure(bg="#121212")

        self.data_queue = queue.Queue()
        
        # --- UI LAYOUT ---
        tk.Label(root, text="BOTTLE DETECTOR: LEFT & RIGHT", font=("Arial", 16, "bold"), bg="#121212", fg="white").pack(pady=10)

        self.frame = tk.Frame(root, bg="#121212")
        self.frame.pack(expand=True, fill="both", padx=20)

        # Left Column
        self.left_label = tk.Label(self.frame, text="LEFT", font=("Arial", 14), bg="#121212", fg="#00ccff")
        self.left_label.grid(row=0, column=0, padx=40)
        self.left_dist = tk.Label(self.frame, text="-- cm", font=("Impact", 40), bg="#121212", fg="#00ccff")
        self.left_dist.grid(row=1, column=0)
        self.left_bar = ttk.Progressbar(self.frame, orient="vertical", length=200, mode="determinate", maximum=40)
        self.left_bar.grid(row=2, column=0, pady=10)

        # Right Column
        self.right_label = tk.Label(self.frame, text="RIGHT", font=("Arial", 14), bg="#121212", fg="#ff33cc")
        self.right_label.grid(row=0, column=1, padx=40)
        self.right_dist = tk.Label(self.frame, text="-- cm", font=("Impact", 40), bg="#121212", fg="#ff33cc")
        self.right_dist.grid(row=1, column=1)
        self.right_bar = ttk.Progressbar(self.frame, orient="vertical", length=200, mode="determinate", maximum=40)
        self.right_bar.grid(row=2, column=1, pady=10)

        # Status
        self.status_var = tk.StringVar(value="OFFLINE")
        tk.Label(root, textvariable=self.status_var, bg="#121212", fg="gray").pack(side="bottom", pady=5)

        self.running = True
        threading.Thread(target=self.bt_worker, daemon=True).start()
        self.process_queue()

    def process_queue(self):
        try:
            while True:
                l_dist, r_dist = self.data_queue.get_nowait()
                self.update_ui(l_dist, r_dist)
        except queue.Empty: pass
        if self.running: self.root.after(20, self.process_queue)

    def update_ui(self, l, r):
        # Update Text
        self.left_dist.config(text=f"{l}cm")
        self.right_dist.config(text=f"{r}cm")
        
        # Update Bars (Closer = Higher Bar)
        self.left_bar['value'] = max(0, 40 - l)
        self.right_bar['value'] = max(0, 40 - r)

        # Visual Danger Feedback
        self.left_dist.config(fg="red" if l < 10 else "#00ccff")
        self.right_dist.config(fg="red" if r < 10 else "#ff33cc")

    def bt_worker(self):
        while self.running:
            sock = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
            try:
                self.status_var.set("CONNECTING...")
                sock.connect((ESP32_MAC_ADDRESS, PORT))
                self.status_var.set("CONNECTED")
                
                buffer = ""
                while self.running:
                    chunk = sock.recv(1024).decode('utf-8')
                    if not chunk: break
                    buffer += chunk
                    if "\n" in buffer:
                        lines = buffer.split("\n")
                        for line in lines[:-1]:
                            if "L:" in line and "R:" in line:
                                try:
                                    parts = line.split(",")
                                    l_val = int(parts[0].replace("L:", ""))
                                    r_val = int(parts[1].replace("R:", ""))
                                    self.data_queue.put((l_val, r_val))
                                except: pass
                        buffer = lines[-1]
            except:
                self.status_var.set("LOST CONNECTION - RETRYING...")
                time.sleep(2)
            finally: sock.close()

if __name__ == "__main__":
    root = tk.Tk()
    DualSensorGUI(root)
    root.mainloop()