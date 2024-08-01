import cv2
import os

def capture_chessboard_images(save_dir='chessboard_images', base_filename='chessboard_image', num_images=5):
    # Create the directory if it doesn't exist
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)

    # Open a connection to the camera (0 is usually the built-in webcam)
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("Error: Could not open camera.")
        return

    print("Press 's' to save an image, 'q' to quit.")

    image_count = 0

    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()
        if not ret:
            print("Error: Failed to capture image.")
            break

        # Display the captured frame
        cv2.imshow('Chessboard Capture', frame)

        # Wait for a key press
        key = cv2.waitKey(1) & 0xFF

        if key == ord('s'):
            if image_count < num_images:
                # Save the captured frame with sequential naming
                save_path = os.path.join(save_dir, f"{base_filename}_{image_count+1}.jpg")
                cv2.imwrite(save_path, frame)
                print(f"Image saved to {save_path}")
                image_count += 1
            else:
                print(f"Captured the maximum number of images ({num_images}).")
                break
        elif key == ord('q'):
            # Quit without saving
            print("Quitting.")
            break

    # Release the camera and close any open windows
    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    capture_chessboard_images(save_dir='chessboard_images', base_filename='chessboard_image', num_images=5)
