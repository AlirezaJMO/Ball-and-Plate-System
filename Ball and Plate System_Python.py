from math import floor
import cv2
import numpy as np
from time import time
import serial
import imutils
from math import trunc
# Define the serial port and baud rate
ser = serial.Serial('COM7', 9600)  # Change 'COM3' to your Arduino's serial port

# Define the lower and upper bounds for the white plate in HSV format
lower_white = np.array([0, 0, 135])
upper_white = np.array([106, 255, 205])

# Define the lower and upper bounds for the orange ball in HSV forma
lower_blue = np.array([89, 147, 0])
upper_blue = np.array([179, 255, 255])

ip_webcam_url = 'http://192.168.137.235:4747/video'
# Create a VideoCapture object to connect to the IP webcam.
cap = cv2.VideoCapture(ip_webcam_url)

roi_x = 261
roi_y = 88
roi_w = 353  # Define the initial width of the ROI
roi_h = 345  # Define the initial height of the ROI

plate_dimensions = None

setPoint_x = 0.0
setPoint_y = 0.0
speed_x = 0.0
speed_y = 0.0

# Define the desired width of the output frame
output_width = 640  # Adjust to your desired width


# Control Loop Properties
frequency = 100.0      # Frequency in Hz
period = 1.0 / frequency        # Time period between loop iterations
start_time = time()

fps = int(cap.get(5))
print("fps:", fps)

previous_ball_x = None
previous_ball_y = None
font_size = 0.5  # Adjust the font size as needed

angle1 = 0
angle2 = -3
ser.write(f"{angle1},{angle2}\n".encode())
while True:
    
    

    ret, frame = cap.read()
    if not ret:
        break
    cropped_frame1 = frame[roi_y:roi_y + roi_h, roi_x:roi_x + roi_w]
    cv2.imshow("Vi", imutils.rotate(cropped_frame1, -90))
    
    # Convert the frame to the HSV color space
    hsv = cv2.cvtColor(cropped_frame1, cv2.COLOR_BGR2HSV)

    # Create a mask to extract the white color
    white_mask = cv2.inRange(hsv, lower_white, upper_white)

    # Apply a series of morphological operations to remove noise for the mask
    white_mask = cv2.erode(white_mask, None, iterations=2)
    white_mask = cv2.dilate(white_mask, None, iterations=2)

    # Find contours for the white color
    white_contours, _ = cv2.findContours(white_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if plate_dimensions is None:
        for contour in white_contours:
            if cv2.contourArea(contour) > 500:  # Adjust the area threshold as needed
                epsilon = 0.03 * cv2.arcLength(contour, True)
                approx = cv2.approxPolyDP(contour, epsilon, True)

                if len(approx) == 4:
                    x, y, w, h = cv2.boundingRect(approx)
                    plate_dimensions = (x, y, w, h)
                    break

    if plate_dimensions is not None:
        x, y, w, h = plate_dimensions

        # Crop the frame to the dimensions of the white plate
        cropped_frame = cropped_frame1[y:y + h, x:x + w]
        cropped_frame = imutils.rotate(cropped_frame, -90)
        # Resize the cropped frame while maintaining the aspect ratio
        aspect_ratio = float(output_width) / w
        new_height = int(h * aspect_ratio)
        cropped_frame = cv2.resize(cropped_frame, (output_width, new_height))

        # Convert the cropped frame to the HSV color space
        hsv_cropped = cv2.cvtColor(cropped_frame, cv2.COLOR_BGR2HSV)

        # Create a mask to extract the orange ball within the cropped frame
        blue_mask = cv2.inRange(hsv_cropped, lower_blue ,upper_blue)

        # Find contours for the orange ball within the cropped frame
        blue_contours, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for blue_contour in blue_contours:
            if cv2.contourArea(blue_contour) > 100:  # Adjust the area threshold as needed
                (x_o, y_o), radius = cv2.minEnclosingCircle(blue_contour)
                center = (int(x_o), int(y_o))
                radius = int(radius)
                cv2.circle(cropped_frame, center, radius, (0, 0, 255), 2)
                
                # Calculate the coordinates of the ball's center within the cropped frame
                ball_x = int(x_o)
                ball_y = int(y_o)

                # Calculate the frame's center
                frame_center_x = cropped_frame.shape[1] // 2
                frame_center_y = cropped_frame.shape[0] // 2

                # Calculate the ball's position relative to the frame's center
                relative_xx = "{:.2f}".format(((ball_x - frame_center_x)*float(0.1 / w))) #
                relative_x = float(relative_xx)
                relative_yy = "{:.2f}".format(-((ball_y - frame_center_y)*float(0.1 / h)))  #  Note: Inverted Y-axis
                relative_y = float(relative_yy)
                # Print the relative coordinates on the output frame
                cv2.putText(cropped_frame, "X: {}".format(relative_x), (10, 20), cv2.FONT_HERSHEY_SIMPLEX, font_size, (0, 255, 0), 1)
                cv2.putText(cropped_frame, "Y: {}".format(relative_y), (10, 40), cv2.FONT_HERSHEY_SIMPLEX, font_size, (0, 255, 0), 1)
                if previous_ball_x is not None and previous_ball_y is not None:
                    delta_x = relative_x - previous_ball_x
                    delta_y = relative_y - previous_ball_y

                    speed_x = (delta_x / period)
                    speed_y = (delta_y / period)

                    # Print the speed with the smaller font
                    cv2.putText(cropped_frame, "Speed X: {:.2f} m/s".format(speed_x), (10, 60), cv2.FONT_HERSHEY_SIMPLEX, font_size, (0, 255, 0), 1)
                    cv2.putText(cropped_frame, "Speed Y: {:.2f} m/s".format(speed_y), (10, 80), cv2.FONT_HERSHEY_SIMPLEX, font_size, (0, 255, 0), 1)

                previous_ball_x = relative_x
                previous_ball_y = relative_y

                k1 = -174.0
                k2 = -8.5
            
                # angle1 = asin(round(max(-5, min(-Kf*setPoint_y-(k1*relative_y + k2*speed_y), 5)), 1)*(pi/180))  
                # angle2 =   asin(round(max(-5, min(-Kf*setPoint_x-(k1*relative_x + k2*speed_x), 5)), 1)*(pi/180))
                angle1 = -2 + round(max(-5, min(-(k1*relative_y + k2*speed_y), 5)), 1)
                angle2 = 2 + round(max(-5, min(-(k1*relative_x + k2*speed_x), 5)), 1)
                print((k1*relative_y + k2*speed_y), (k1*relative_x + k2*speed_x), angle1, angle2)
                # Send angles to the Arduino
                ser.write(f"{angle1},{angle2}\n".encode())

        # Draw a green rectangle around the white plate
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

        cv2.imshow("Cropped Video with Ball Detection", cropped_frame)
    while time() - start_time < period:
    
        pass

    start_time = time()

    if cv2.waitKey(1) & 0xFF == 27:  # Press 'Esc' to exit
        break

cap.release()
cv2.destroyAllWindows()
