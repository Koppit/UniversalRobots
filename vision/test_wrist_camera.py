import cv2
import requests
import numpy as np

# Replace with your actual Universal Robot IP address
ROBOT_IP = "192.168.0.25" 
URL = f"http://{ROBOT_IP}:4242/current.jpg?type=color"

print("Press 'q' to exit the stream.")

while True:
    try:
        # Fetch the current image from the Robotiq vision server
        response = requests.get(URL, timeout=1)
        
        if response.status_code == 200:
            # Convert raw bytes into a numpy array for OpenCV
            img_array = np.asarray(bytearray(response.content), dtype=np.uint8)
            frame = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
            
            # Display the stream
            cv2.imshow("Robotiq Wrist Camera Feed", frame)
        else:
            print(f"Failed to fetch image. Status code: {response.status_code}")

    except requests.exceptions.RequestException as e:
        print(f"Connection error: {e}")
        break

    # Press 'q' to quit the loop
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()