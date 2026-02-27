import time
from picamera2 import Picamera2

def test_camera_stream():
    # Initialize Picamera2
    # The Raspberry Pi 5 handles the hardware synchronization automatically
    pc2 = Picamera2()

    try:
        print("Configuring Camera Module 3...")
        # Configure the camera for a standard 720p preview stream
        config = pc2.create_preview_configuration(main={"size": (1280, 720)})
        pc2.configure(config)

        print("Starting stream preview...")
        # This opens a window on your Pi's desktop (X11 or Wayland)
        pc2.start_preview()

        print("Stream active. Press Ctrl+C to stop.")
        
        # Keep the script running so the preview stays open
        while True:
            time.sleep(1)

    except KeyboardInterrupt:
        print("\nStopping stream...")
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        # Ensure the camera resources are released properly
        pc2.stop_preview()
        pc2.close()
        print("Camera closed.")

if __name__ == "__main__":
    test_camera_stream()