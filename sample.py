import socket
import sys

# ==========================================================
# CONFIGURATION - UPDATE THE MAC ADDRESS BELOW
# ==========================================================
# Run 'hcitool scan' or 'bluetoothctl scan on' to find yours
ESP32_MAC_ADDRESS = "XX:XX:XX:XX:XX:XX" 
PORT = 1 # Standard RFCOMM port for ESP32 Bluetooth Serial

def run_bluetooth_client():
    # Create the RFCOMM socket
    # We use AF_BLUETOOTH for Bluetooth communication
    try:
        sock = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
        
        print(f"Attempting to connect to ESP32 ({ESP32_MAC_ADDRESS})...")
        
        # Connect to the ESP32
        sock.connect((ESP32_MAC_ADDRESS, PORT))
        print("Connected successfully!")
        print("Type your message and press Enter. Type 'exit' to quit.")

        while True:
            # Get input from the user
            message = input("You (Pi): ")
            
            if message.lower() == 'exit':
                print("Closing connection...")
                break

            # Send the data to the ESP32
            # We add \n because the ESP32 code looks for a newline
            sock.send(bytes(message + "\n", 'utf-8'))

            # Wait for a response from the ESP32
            data = sock.recv(1024)
            if data:
                print(f"ESP32: {data.decode('utf-8').strip()}")

    except socket.error as e:
        print(f"Couldn't connect to Bluetooth device: {e}")
    except KeyboardInterrupt:
        print("\nScript stopped by user.")
    finally:
        sock.close()
        print("Socket closed.")

if __name__ == "__main__":
    if ESP32_MAC_ADDRESS == "XX:XX:XX:XX:XX:XX":
        print("ERROR: Please edit the script and enter your ESP32's MAC address!")
        sys.exit()
    run_bluetooth_client()