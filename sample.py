import socket
import time
import sys

# ==========================================================
# CONFIGURATION - UPDATED WITH YOUR NEW SCAN RESULT
# ==========================================================
ESP32_MAC_ADDRESS = "F4:2D:C9:71:C9:E2" 
PORT = 1 

def start_robot_comms():
    print(f"--- AUTONOMOUS PET BOTTLE COLLECTOR INTERFACE ---")
    print(f"Target Device: {ESP32_MAC_ADDRESS}")
    
    while True:
        # Create a new Bluetooth RFCOMM socket
        sock = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
        
        try:
            print(f"\n[*] Attempting to connect...")
            # 10-second timeout for the initial handshake
            sock.settimeout(10) 
            sock.connect((ESP32_MAC_ADDRESS, PORT))
            
            # Connection successful
            sock.settimeout(None) 
            print("[+] Status: CONNECTED")
            print("[i] Type 'exit' to quit or send a command (e.g., FORWARD, STOP)")

            while True:
                # 1. Get Command from User
                message = input("Robot CMD >> ")
                
                if message.lower() == 'exit':
                    print("[!] Shutting down interface...")
                    sock.close()
                    return 

                if not message:
                    continue

                # 2. Send command to ESP32
                # Adding \n ensures the ESP32 knows the message is complete
                sock.send(bytes(message + "\n", 'utf-8'))

                # 3. Receive Feedback (Non-blocking check)
                sock.settimeout(1.0) # Wait up to 1 second for a reply
                try:
                    data = sock.recv(1024)
                    if data:
                        print(f"Robot Response: {data.decode('utf-8').strip()}")
                except socket.timeout:
                    # No response yet, which is fine for some commands
                    pass
                finally:
                    sock.settimeout(None)

        except (socket.error, socket.timeout) as e:
            print(f"[-] Connection Error: {e}")
            print("[*] Retrying in 3 seconds... (Check if ESP32 is powered)")
            sock.close()
            time.sleep(3) 
            
        except KeyboardInterrupt:
            print("\n[!] User interrupted via keyboard. Closing.")
            sock.close()
            sys.exit()

if __name__ == "__main__":
    start_robot_comms()