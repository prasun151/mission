import cv2
import datetime

def scan_and_save():
    # Initialize webcam (0 is default)
    cap = cv2.VideoCapture(0)
    detector = cv2.QRCodeDetector()
    
    last_data = None
    filename = "qr_data.txt"

    print(f"Scanning... Data will be saved to {filename}")
    print("Press 'q' to quit.")

    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        # Detect and decode
        data, vertices, _ = detector.detectAndDecode(frame)

        if data:
            # Visual feedback: Draw box around QR code
            if vertices is not None:
                frame = cv2.polylines(frame, [vertices.astype(int)], True, (0, 255, 0), 2)

            # Save only if it's new data to avoid spamming the file
            if data != last_data:
                timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                output_line = f"[{timestamp}] {data}\n"
                
                with open(filename, "a") as f:
                    f.write(output_line)
                
                print(f"Captured: {data}")
                last_data = data

        cv2.imshow("QR Scanner", frame)

        # Exit loop when 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    scan_and_save()