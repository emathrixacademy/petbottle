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

class RealTimeRobotGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("LIVE: Bottle Collector Sensor")
        self.root.geometry("450x350")
        self.root.configure(bg="#1e1e1e") # Dark theme for better contrast

        self.data_queue = queue.Queue()
        
        # --- UI DESIGN ---
        self.label_title = tk.Label(root, text="REAL-TIME PROXIMITY", font=("Courier", 18, "bold"), bg="#1e1e1e", fg="#00ff00")
        self.label_title.pack(pady=15)

        # Big Digital Display
        self.dist_var = tk.StringVar(value="-- cm")
        self.dist_label = tk.Label(root, textvariable=self.dist_var, font=("Impact", 60), bg="#1e1e1e", fg="#00ff00")
        self.dist_label.pack()

        # Smooth Progress Bar
        self.style = ttk.Style()
        self.style.theme_use('default')
        self.style.configure("TProgressbar", thickness=30, troughcolor='#333', background='#00ff00')
        
        self.progress = ttk.Progressbar(root, orient="horizontal", length=350, mode="determinate", maximum=40)
        self.progress.pack(pady=20)

        self.status_var = tk.StringVar(value="SEARCHING FOR ROBOT...")
        self.status_label = tk.Label(root, textvariable=self.status_var, font=("Arial", 10), bg="#1e1e1e", fg="white")
        self.status_label.pack(side="bottom", pady=10)

        # Start Background Process
        self.running = True
        self.bt_thread = threading.Thread(target=self.bluetooth_worker, daemon=True)
        self.bt_thread.start()
        
        # Start UI Update Loop
        self.process_queue()

    def process_queue(self):
        """ Checks the queue for new data and updates UI immediately """
        try:
            while True:
                distance = self.data_queue.get_nowait()
                self.update_display(distance)
        except queue.Empty:
            pass
        
        if self.running:
            self.root.after(20, self.process_queue) # Refresh every 20ms (50 FPS)

    def update_display(self, dist):
        self.dist_var.set(f"{dist} cm")
        
        # Animated Bar Logic (40cm scale)
        val = 40 - dist
        self.progress['value'] = max(0, val)

        # Dynamic Color Shift (Green -> Red)
        if dist < 8:
            self.dist_label.config(fg="#ff3333")
            self.style.configure("TProgressbar", background='#ff3333')
            self.root.configure(bg="#2b0000") # Subtle red background
        elif dist < 20:
            self.dist_label.config(fg="#ffcc00")
            self.style.configure("TProgressbar", background='#ffcc00')
            self.root.configure(bg="#1e1e1e")
        else:
            self.dist_label.config(fg="#00ff00")
            self.style.configure("TProgressbar", background='#00ff00')
            self.root.configure(bg="#1e1e1e")

    def bluetooth_worker(self):
        """ Background thread to handle Bluetooth without lagging the UI """
        while self.running:
            sock = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
            try:
                self.root.after(0, lambda: self.status_var.set("ATTEMPTING CONNECTION..."))
                sock.connect((ESP32_MAC_ADDRESS, PORT))
                self.root.after(0, lambda: self.status_var.set("CONNECTED - STREAMING LIVE"))
                
                buffer = ""
                while self.running:
                    data = sock.recv(1024).decode('utf-8')
                    if not data: break
                    
                    buffer += data
                    if "\n" in buffer:
                        lines = buffer.split("\n")
                        for line in lines[:-1]:
                            if "D:" in line:
                                try:
                                    val = int(line.replace("D:", "").strip())
                                    self.data_queue.put(val)
                                except: pass
                        buffer = lines[-1]
            except:
                self.root.after(0, lambda: self.status_var.set("CONNECTION LOST - RETRYING..."))
                time.sleep(2)
            finally:
                sock.close()

if __name__ == "__main__":
    root = tk.Tk()
    gui = RealTimeRobotGUI(root)
    
    def on_exit():
        gui.running = False
        root.destroy()
        
    root.protocol("WM_DELETE_WINDOW", on_exit)
    root.mainloop()