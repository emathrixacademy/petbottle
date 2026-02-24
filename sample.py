import socket
import time
import sys
import select

# ==========================================================
# CONFIGURATION - YOUR ESP32 MAC ADDRESS
# ==========================================================
ESP32_MAC_ADDRESS = "F4:2D:C9:71:C9:E2" 
PORT = 1 

def start_robot_monitor():
    print("==============================================")
    print("   PET BOTTLE COLLECTOR - SENSOR MONITOR      ")
    print("==============================================")
    
    while True:
        # Create Bluetooth RFCOMM socket
        sock = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
        
        try:
            print(f"[*] Connecting to {ESP32_MAC_ADDRESS}...")
            sock.settimeout(10)
            sock.connect((ESP32_MAC_ADDRESS, PORT))
            sock.settimeout(None)
            print("[+] Status: CONNECTED")
            print("[i] Monitoring for bottles... (Type 'BEEP' to test buzzer)")
            print("----------------------------------------------")

            while True:
                # Use select to check if there is data from the ESP32
                # This prevents the script from freezing while waiting for input
                rlist, _, _ = select.select([sock, sys.stdin], [], [], 0.1)

                for ready_source in rlist:
                    # Case 1: Data coming from ESP32 (Sensor Alert)
                    if ready_source == sock:
                        data = sock.recv(1024)
                        if data:
                            msg = data.decode('utf-8').strip()
                            if "BOTTLE_DETECTED" in msg:
                                # Extract distance from "BOTTLE_DETECTED:8"
                                try:
                                    dist = msg.split(":")[1]
                                    print(f"!!! [ALERT] Bottle Found at {dist} cm !!!")
                                except IndexError:
                                    print(f"[RAW]: {msg}")
                            else:
                                print(f"[ESP32]: {msg}")
                        else:
                            # If recv returns empty, connection is broken
                            raise socket.error("Remote side closed connection")

                    # Case 2: User typing in the terminal
                    elif ready_source == sys.stdin:
                        user_cmd = sys.stdin.readline().strip().upper()
                        if user_cmd == "EXIT":
                            sock.close()
                            return
                        if user_cmd:
                            sock.send(bytes(user_cmd + "\n", 'utf-8'))
                            print(f"[*] Command Sent: {user_cmd}")

        except (socket.error, socket.timeout) as e:
            print(f"[-] Connection Lost: {e}")
            print("[*] Retrying in 3 seconds...")
            sock.close()
            time.sleep(3)
            
        except KeyboardInterrupt:
            print("\n[!] Application stopped by user.")
            sock.close()
            sys.exit()

if __name__ == "__main__":
    start_robot_monitor()