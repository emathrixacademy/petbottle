import socket
import time
import sys
import select

# ==========================================================
# CONFIGURATION - YOUR ESP32 MAC ADDRESS
# ==========================================================
ESP32_MAC_ADDRESS = "F4:2D:C9:71:C9:E2" 
PORT = 1 

def start_proximity_monitor():
    print("==============================================")
    print("   PET BOTTLE COLLECTOR - PROXIMITY ALERTS    ")
    print("==============================================")
    
    while True:
        sock = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
        
        try:
            print(f"[*] Connecting to {ESP32_MAC_ADDRESS}...")
            sock.settimeout(10)
            sock.connect((ESP32_MAC_ADDRESS, PORT))
            sock.settimeout(None)
            print("[+] Status: LIVE MONITORING")
            print("----------------------------------------------")

            while True:
                rlist, _, _ = select.select([sock, sys.stdin], [], [], 0.1)

                for ready_source in rlist:
                    if ready_source == sock:
                        data = sock.recv(1024)
                        if data:
                            msg = data.decode('utf-8').strip()
                            if "PROXIMITY" in msg:
                                try:
                                    dist = int(msg.split(":")[1])
                                    # Create a simple visual bar
                                    bar = "#" * (30 - dist)
                                    print(f"Distance: {dist:3}cm | {bar.ljust(30)} |", end='\r')
                                    
                                    if dist < 5:
                                        print("\n!!! CRITICAL PROXIMITY - BOTTLE DETECTED !!!")
                                except:
                                    pass
                        else:
                            raise socket.error("Disconnected")

                    elif ready_source == sys.stdin:
                        cmd = sys.stdin.readline().strip().upper()
                        if cmd == "EXIT": return
                        sock.send(bytes(cmd + "\n", 'utf-8'))

        except (socket.error, socket.timeout):
            print("\n[-] Connection Lost. Retrying...")
            sock.close()
            time.sleep(3)
        except KeyboardInterrupt:
            sock.close()
            sys.exit()

if __name__ == "__main__":
    start_proximity_monitor()