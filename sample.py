import socket
import time
import sys

# ==========================================================
# CONFIGURATION - USE YOUR SCANNED MAC ADDRESS
# ==========================================================
ESP32_MAC_ADDRESS = "F4:2D:C9:71:C9:E2" 
PORT = 1 

def start_collector_interface():
    print("==============================================")
    print("   PET BOTTLE COLLECTOR - 5V CONTROL PANEL    ")
    print("==============================================")
    
    while True:
        # Initialize Bluetooth Socket
        sock = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
        
        try:
            print(f"\n[*] Connecting to Robot ({ESP32_MAC_ADDRESS})...")
            sock.settimeout(10)
            sock.connect((ESP32_MAC_ADDRESS, PORT))
            sock.settimeout(None)
            print("[+] Status: CONNECTED")
            print("[!] Note: Running on 5V. Use small degree steps if motor stalls.")
            print("----------------------------------------------")

            while True:
                # 1. User Input
                cmd = input("Enter Degrees (e.g. 90, -45) or 'OFF': ").strip()
                
                if cmd.lower() == 'exit':
                    sock.close()
                    return
                
                if not cmd:
                    continue

                # 2. Send Command
                print(f"[*] Sending: {cmd}")
                sock.send(bytes(cmd + "\n", 'utf-8'))

                # 3. Wait for Robot Response (Confirmation of movement)
                # We give it a longer timeout (5s) because 5V motors move slower
                sock.settimeout(5.0)
                try:
                    data = sock.recv(1024)
                    if data:
                        response = data.decode('utf-8').strip()
                        print(f"[Robot]: {response}")
                        
                        # Wait until the 'Done' message arrives before sending more
                        if "Moving" in response:
                            # Listen again for the 'Done.' confirmation
                            done_data = sock.recv(1024)
                            print(f"[Robot]: {done_data.decode('utf-8').strip()}")
                
                except socket.timeout:
                    print("[!] Warning: Robot is taking a long time to respond...")
                finally:
                    sock.settimeout(None)

        except (socket.error, socket.timeout) as e:
            print(f"[-] Connection Lost: {e}")
            print("[*] Attempting to reconnect in 3 seconds...")
            sock.close()
            time.sleep(3)
            
        except KeyboardInterrupt:
            print("\n[!] Closing Application.")
            sock.close()
            sys.exit()

if __name__ == "__main__":
    start_collector_interface()