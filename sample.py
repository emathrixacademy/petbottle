import socket
import time
import sys

# ==========================================================
# CONFIGURATION - UPDATED WITH YOUR DEVICE MAC
# ==========================================================
ESP32_MAC_ADDRESS = "00:4B:12:30:06:FA" 
PORT = 1 

def start_robot_comms():
    print(f"--- Autonomous Pet Bottle Collector Interface ---")
    
    while True:
        # Create a new Bluetooth RFCOMM socket
        sock = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
        
        try:
            print(f"Connecting to ESP32_Chat ({ESP32_MAC_ADDRESS})...")
            # Set a 10-second timeout for the connection attempt
            sock.settimeout(10) 
            sock.connect((ESP32_MAC_ADDRESS, PORT))
            
            # Connection successful, disable timeout for continuous streaming
            sock.settimeout(None) 
            print("Status: CONNECTED")
            print("Commands: Type your message or 'exit' to quit.")

            while True:
                # 1. Send data to Robot
                message = input("Command to Robot >> ")
                
                if message.lower() == 'exit':
                    print("Shutting down interface...")
                    sock.close()
                    return 

                # Send message with newline for ESP32 parsing
                sock.send(bytes(message + "\n", 'utf-8'))

                # 2. Receive data from Robot (e.g., sensor updates)
                # Using a small timeout here so the script doesn't hang if ESP32 is silent
                sock.settimeout(2.0)
                try:
                    data = sock.recv(1024)
                    if data:
                        print(f"Robot Response: {data.decode('utf-8').strip()}")
                except socket.timeout:
                    # It's okay if the robot doesn't reply immediately
                    pass
                finally:
                    sock.settimeout(None)

        except (socket.error, socket.timeout) as e:
            print(f"Connection Error: {e}")
            print("Retrying in 3 seconds... (Check if ESP32 is powered on)")
            sock.close()
            time.sleep(3) 
            
        except KeyboardInterrupt:
            print("\nUser interrupted. Closing.")
            sock.close()
            sys.exit()

if __name__ == "__main__":
    start_robot_comms()