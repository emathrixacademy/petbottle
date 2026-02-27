import time
import sys
import io

try:
    from picamera2 import Picamera2
    print("Picamera2 library loaded successfully!")
except ImportError:
    print("Error: picamera2 still not found in this environment.")
    print("Ensure you created the venv with --system-site-packages")
    sys.exit(1)

def run_test():
    # Initialize Picamera2 for Camera Module 3
    picam2 = Picamera2()
    
    try:
        # Request a configuration for the preview
        # Note: Pi 5 handles the 'imx708' (Module 3) automatically
        config = picam2.create_preview_configuration(main={"size": (1280, 720)})
        picam2.configure(config)
        
        print("Starting camera preview window...")
        print("Note: This requires a monitor connected to the Pi or VNC.")
        
        picam2.start_preview()
        
        print("Stream active. Press Ctrl+C to exit.")
        
        while True:
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("\nStopping...")
    except Exception as e:
        print(f"Hardware Error: {e}")
    finally:
        picam2.stop_preview()
        picam2.close()
        print("Camera resources released.")

if __name__ == "__main__":
    run_test()