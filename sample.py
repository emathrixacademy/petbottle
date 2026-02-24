import socket
import threading
import tkinter as tk
from tkinter import ttk
import sys

# ==========================================================
# CONFIGURATION - YOUR ESP32 MAC ADDRESS
# ==========================================================
ESP32_MAC_ADDRESS = "F4:2D:C9:71:C9:E2" 
PORT = 1 

class RobotGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Autonomous Bottle Collector - Monitor")
        self.root.geometry("400x300")
        
        # --- UI Elements ---
        self.label_title = tk.Label(root, text="Robot Proximity Monitor", font=("Arial", 16, "bold"))
        self.label_title.pack(pady=10)

        self.status_label = tk.Label(root, text="Status: Disconnected", fg="red")
        self.status_label.pack()

        # Distance Display
        self.dist_text = tk.Label(root, text="Distance: -- cm", font=("Arial", 24))
        self.dist_text.pack(pady=20)

        # Progress Bar (Visual Meter)
        self.style = ttk.Style()
        self.progress = ttk.Progressbar(root, orient="horizontal", length=300, mode="determinate", maximum=30)
        self.progress.pack(pady=10)

        # Warning Label
        self.warning_label = tk.Label(root, text="", font=("Arial", 12, "bold"))
        self.warning_label.pack(pady=10)

        # Manual Beep Button
        self.beep_btn = tk.Button(root, text="TEST BUZZER", command=self.send_beep, width=20, height=2)
        self.beep_btn.pack(pady=10)

        # Bluetooth Variables
        self.sock = None
        self.running = True

        # Start Bluetooth Thread
        self.bt_thread = threading.Thread(target=self.bluetooth_loop, daemon=True)
        self.bt_thread.start()

    def send_beep(self):
        if self.sock:
            try:
                self.sock.send(bytes("BEEP\n", 'utf-8'))
            except:
                pass

    def update_ui(self, distance):
        # Update Text
        self.dist_text.config(text=f"Distance: {distance} cm")
        
        # Update Progress Bar (Inverted so closer = fuller bar)
        val = 30 - distance
        self.progress['value'] = max(0, val)

        # Update Colors and Warnings
        if distance < 7:
            self.dist_text.config(fg="red")
            self.warning_label.config(text="!!! BOTTLE DETECTED !!!", fg="red")
            self.root.configure(bg="#ffe6e6") # Light red flash
        elif distance < 15:
            self.dist_text.config(fg="#cc9900") # Gold/Yellow
            self.warning_label.config(text="Object Approaching...", fg="#cc9900")
            self.root.configure(bg="white")
        else:
            self.dist_text.config(fg="green")
            self.warning_label.config(text="Clear", fg="green")
            self.root.configure(bg="white")

    def bluetooth_loop(self):
        while self.running:
            self.sock = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
            try:
                self.status_label.config(text=f"Connecting to {ESP32_MAC_ADDRESS}...", fg="orange")
                self.sock.connect((ESP32_MAC_ADDRESS, PORT))
                self.status_label.config(text="Status: CONNECTED", fg="green")
                
                while self.running:
                    data = self.sock.recv(1024)
                    if data:
                        msg = data.decode('utf-8').strip()
                        if "PROXIMITY" in msg:
                            try:
                                d = int(msg.split(":")[1])
                                # Schedule UI update in the main thread
                                self.root.after(0, self.update_ui, d)
                            except:
                                pass
                    else:
                        break
            except Exception as e:
                self.status_label.config(text="Status: Connection Failed. Retrying...", fg="red")
                time.sleep(3)
            finally:
                self.sock.close()

if __name__ == "__main__":
    root = tk.Tk()
    app = RobotGUI(root)
    
    def on_closing():
        app.running = False
        root.destroy()
        sys.exit()

    root.protocol("WM_DELETE_WINDOW", on_closing)
    root.mainloop()