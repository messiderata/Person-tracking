from ultralytics import YOLO
from pyfirmata import Arduino, util
import threading
import time
import customtkinter as ctk

# Load your YOLOv5 model
model = YOLO('Actual.pt')

# Set up the Firmata connection to the Arduino board
board = Arduino('COM6')  # Update with your port

# Start an iterator thread so that serial buffer doesn't overflow
it = util.Iterator(board)
it.start()

# Define the pin for the servo motor
servo_pin = 9  # Change this to the pin number connected to your servo
servo = board.get_pin('d:{}:s'.format(servo_pin))

# Define the center position of the camera
camera_center = 320  # Assuming the camera resolution width is 640 pixels

# Define the proportional, integral, and derivative gains (adjust as needed)
Kp = 0.005
Ki = 0.01
Kd = 0.05

# Counter to track consecutive frames without detecting any object
no_object_count = 0

# Initialize variables for PID controller
prev_error = 0
integral = 0

# Function to run inference and servo control
def run_inference_and_servo_control(source, conf):
    global no_object_count, prev_error, integral
    
    while True:
        results = model.predict(source=source, save=False, imgsz=640, conf=conf, show=True, stream=True)
        object_detected = False  # Flag to indicate if any object is detected in the current frame
        
        for r in results:
            boxes = r.boxes
            bounding_boxes = boxes.xyxy.tolist()
            
            if bounding_boxes:
                no_object_count = 0
                
                # Get the bounding box of the first detected object
                bbox = bounding_boxes[0]
                x_center = (bbox[0] + bbox[2]) / 2  # Calculate the x-coordinate of the object center
                
                # Calculate the error between the object center and the camera center
                error = x_center - camera_center
                
                # Update integral and derivative terms
                integral += error
                derivative = error - prev_error
                
                # Calculate the servo adjustment using PID control
                servo_adjustment = Kp * error + Ki * integral + Kd * derivative
                
                # Set the new servo angle based on the adjustment
                servo_angle = 90 - int(servo_adjustment)
                
                # Make sure the servo angle is within the valid range
                servo_angle = max(0, min(180, servo_angle))
                
                # Move the servo to the calculated angle
                servo.write(servo_angle)
                print("Servo angle:", servo_angle)
                
                # Update previous error
                prev_error = error
                
                object_detected = True
        
        if not object_detected:
            # Increment the counter if no object is detected
            no_object_count += 1
            
            # If no object is detected for a prolonged period, reset the servo to center position
            if no_object_count >= 20:
                servo.write(90)  # Move the servo to the middle position
                print("No object detected for 20 consecutive frames. Stopping servo at 90 degrees.")
                no_object_count = 0  # Reset the counter
        
        # Add a small delay to avoid excessive servo movements
        time.sleep(1)

# --- GUI Section ---
class App(ctk.CTk):
    def __init__(self):
        super().__init__()
        my_font = ctk.CTkFont(family="Satoshi-Bold", size=14)

        self.title("Object Detection and Servo Control")
        self.geometry("400x300")
        self.configure(fg_color="#20242a") 
        self.center_window()

        # --- Frames for GUI Elements ---
        button_frame = ctk.CTkFrame(master=self, fg_color="transparent")
        button_frame.pack(pady=10, padx=10)

        button_frame2 = ctk.CTkFrame(master=self, fg_color="transparent")
        button_frame2.pack(pady=5, padx=10)

        # --- Labels and ComboBoxes ---
        self.result_label = ctk.CTkLabel(
            master=button_frame,
            text="Source:",
            fg_color="transparent",
            text_color="white",
            font=my_font
        )
        self.result_label.pack(pady=5, padx=10, expand=True)

        self.result_label1 = ctk.CTkLabel(
            master=button_frame2,
            text="Confidence:",
            fg_color="transparent",
            text_color="white",
            font=my_font
        )
        self.result_label1.pack(pady=5, padx=10, expand=True)

        self.optionmenu_1 = ctk.CTkComboBox(
            button_frame,
            fg_color="#fed32c",
            text_color="#20242a",
            button_color="#fed32c",
            border_color="#fed32c",
            font=my_font,
            values=["0", "1", "2"]
        )
        self.optionmenu_1.pack(pady=10)

        self.optionmenu_2 = ctk.CTkComboBox(
            button_frame2,
            fg_color="#fed32c",
            text_color="#20242a",
            button_color="#fed32c",
            border_color="#fed32c",
            font=my_font,
            values=["0.25", "0.50", "0.75", "1.00"]
        )
        self.optionmenu_2.pack(pady=10)

        self.button1 = ctk.CTkButton(
            master=button_frame2,
            text="START",
            command=self.print_combo_box,
            fg_color="white",
            hover_color="#fede5f",
            text_color="#20242a",
            font=my_font
        )
        self.button1.pack(pady=5, padx=10, side="bottom")

    def print_combo_box(self):
        source = self.optionmenu_1.get()
        conf = float(self.optionmenu_2.get())
        Run(source, conf)

    def center_window(self):
        screen_width = self.winfo_screenwidth()
        screen_height = self.winfo_screenheight()
        x = (screen_width - self.winfo_reqwidth()) // 2
        y = (screen_height - self.winfo_reqheight()) // 2
        self.geometry(f"+{x}+{y}")


def Run(source, conf):
    # Create and start the thread
    inference_thread = threading.Thread(target=run_inference_and_servo_control, args=(source, conf))
    inference_thread.daemon = True  # Daemonize the thread so it automatically closes when the main program exits
    inference_thread.start()

    # Optionally return the thread object if needed
    return inference_thread


if __name__ == "__main__":
    app = App()
    app.mainloop()
