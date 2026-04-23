import cv2
import numpy as np
from ultralytics import YOLO
import pygame
import threading
import time
import rospy
import signal
import sys
from geometry_msgs.msg import Twist # Kept from original
from std_msgs.msg import Bool, Float32, String # Added String
from flask import Flask, Response, render_template, jsonify # Added jsonify
import json # Added for JSON

# --- Flask App Setup --- (Original)
app = Flask(__name__)
latest_processed_frame = None
frame_lock = threading.Lock()

# --- ADDED: Global variables for dashboard data ---
dashboard_data_lock = threading.Lock() # Lock for both battery and robot status
latest_battery_info = {
    "voltage": 0.0,
    "rail_5v": 0.0,
    "percentage": 0.0
}
latest_robot_status_info = { # Default structure matching Go_To_Goal
    "status_msg": "Waiting for data...",
    "distance": 0.0,
    "angle_diff": 0.0,
    "current_target_x": 0.0,
    "current_target_y": 0.0,
    "original_goal_x": 0.0,
    "original_goal_y": 0.0,
    "goal_reached_msg": "",
    "robot_stopped_text": ""
}
# ---

# Initialize pygame mixer (Original)
pygame.mixer.init()
fire_alarm_sound = pygame.mixer.Sound('/usr/share/sounds/freedesktop/stereo/suspend-error.oga')
emergency_alarm_sound = pygame.mixer.Sound('/usr/share/sounds/freedesktop/stereo/alarm-clock-elapsed.oga')

# Global variables (Original)
stop_fire_alarm = False
alarm_thread = None
running = True 
flask_server = None # Not actively used but kept from original

# Detection state variables (Original)
fire_start_time = None
fire_alarm_active = False
combined_start_time = None
combined_detection_threshold = 2
combined_alarm_active = False
person_detected = False # This seems to be per frame in the original logic
fire_detection_threshold = 2

# Signal handler (Original)
def signal_handler(signum, frame):
    global running, stop_fire_alarm 
    print("\nSignal received. Cleaning up...")
    running = False
    stop_all_alarms() # Original function name
    cv2.destroyAllWindows()
    pygame.mixer.quit()
    if 'cap' in globals() and cap.isOpened(): # Original cap variable
        cap.release()
    # rospy.signal_shutdown("Ctrl+C in comb.py") # Optional: if you want ROS node to also know
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)

# Original alarm functions
def play_fire_alarm():
    try:
        while not stop_fire_alarm and running:
            fire_alarm_sound.play()
            # Original logic for sleep/wait
            time.sleep(fire_alarm_sound.get_length() if fire_alarm_sound.get_length() > 0 else 0.5)
            if stop_fire_alarm or not running:
                fire_alarm_sound.stop()
                break
    except Exception as e:
        print(f"Error playing fire alarm: {str(e)}")

def play_emergency_alarm():
    try:
        emergency_alarm_sound.play() # Plays once
    except Exception as e:
        print(f"Error playing emergency alarm: {str(e)}")

def stop_all_alarms(): # Original name
    global stop_fire_alarm, alarm_thread
    stop_fire_alarm = True
    if pygame.mixer.get_busy():
        pygame.mixer.stop()
    if alarm_thread and alarm_thread.is_alive():
        alarm_thread.join(timeout=1.0)
        alarm_thread = None


# Initialize ROS node (Original, ensure only one init_node)
# Note: If this script is the primary ROS node runner, this is fine.
# If Go_To_Goal.py also calls init_node, ensure anonymous=True for one of them
# or ensure they have different node names. Given Go_To_Goal.py has init_node,
# this one should also be anonymous or have a distinct name if not already.
# The original file had `rospy.init_node('obstacle_detector')`
# We'll keep it as is, assuming user manages node uniqueness or anonymity.
if not rospy.core.is_initialized(): # Check if already initialized
    rospy.init_node('obstacle_detector_and_dashboard_bridge', anonymous=True) # Make anonymous to be safe
else:
    rospy.logwarn("ROS node already initialized. `comb.py` will use existing node.")

obstacle_detected_pub = rospy.Publisher('obstacle_detected', Bool, queue_size=1)
obstacle_side_pub = rospy.Publisher('obstacle_side', Float32, queue_size=1)
alarm_active_pub = rospy.Publisher('alarm_active', Bool, queue_size=1) # Original

# Load models (Original)
fire_model = YOLO("models/best_fire_detection.pt")
person_model = YOLO("yolo11n.pt") 
fire_class_names = fire_model.model.names
person_class_names = person_model.names # Original had person_model.names

# Camera parameters (Original)
FOCAL_LENGTH = 500 
BOTTLE_WIDTH = 0.07
frame_width = 640 # Used for frame_center_x
frame_center_x = frame_width // 2

def estimate_distance(pixel_width): # Original
    if pixel_width <= 0: return float('inf')
    return (BOTTLE_WIDTH * FOCAL_LENGTH) / pixel_width

def publish_alarm_status(is_active): # Original
    alarm_msg = Bool(); alarm_msg.data = is_active
    alarm_active_pub.publish(alarm_msg)

# --- ADDED: ROS Subscriber Callbacks for Dashboard Data ---
def battery_voltage_callback(msg):
    global latest_battery_info # Use the new global var
    battery_voltage = msg.data
    # Replicate calculations from battery_display.py
    rail_5v = battery_voltage / 2.4 
    percentage = ((battery_voltage - 10.5) / (12.6 - 10.5)) * 100
    percentage = max(0, min(100, percentage))

    with dashboard_data_lock:
        latest_battery_info["voltage"] = round(battery_voltage, 2)
        latest_battery_info["rail_5v"] = round(rail_5v, 2)
        latest_battery_info["percentage"] = round(percentage, 1)

def robot_dashboard_status_callback(msg):
    global latest_robot_status_info # Use the new global var
    try:
        data = json.loads(msg.data)
        with dashboard_data_lock:
            # Update only if keys exist to prevent errors if Go_To_Goal sends partial data
            for key in latest_robot_status_info.keys():
                if key in data:
                    latest_robot_status_info[key] = data[key]
    except json.JSONDecodeError as e:
        rospy.logwarn(f"Failed to decode robot status JSON in comb.py: {e} - Data: {msg.data}")
    except Exception as e:
        rospy.logerr(f"Error in robot_dashboard_status_callback: {e}")

# --- ADDED: Subscribe to topics ---
rospy.Subscriber('robot/battery_voltage', Float32, battery_voltage_callback) # From main.cpp
rospy.Subscriber('/robot_dashboard_status', String, robot_dashboard_status_callback) # From Go_To_Goal.py
# ---

# Original fire detection handler (kept as is)
def handle_fire_detection(frame_arg, fire_start_time_arg, fire_alarm_active_arg, person_detected_arg):
    # Use _arg suffix to avoid collision with globals if any, though original didn't
    global stop_fire_alarm, alarm_thread # Ensure access to correct globals
    fire_detected_this_frame = False # Local to this call
    
    # Original fire detection logic using frame_arg, fire_start_time_arg etc.
    fire_results = fire_model.track(frame_arg, conf=0.5, persist=True, verbose=False) # Added verbose=False
    if fire_results[0].boxes is not None and len(fire_results[0].boxes) > 0:
        boxes = fire_results[0].boxes.xyxy.int().cpu().tolist()
        class_ids = fire_results[0].boxes.cls.int().cpu().tolist()

        for box, class_id in zip(boxes, class_ids):
            class_name = fire_class_names[class_id].lower()
            if class_name == "fire":
                fire_detected_this_frame = True
                x1, y1, x2, y2 = box
                cv2.rectangle(frame_arg, (x1, y1), (x2, y2), (0, 0, 255), 2)
                cv2.putText(frame_arg, "Fire", (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                
                if fire_start_time_arg is None:
                    fire_start_time_arg = time.time()
                    # stop_fire_alarm = False # Original might have this here
                
                # Original alarm logic:
                elif time.time() - fire_start_time_arg >= fire_detection_threshold and \
                     not fire_alarm_active_arg and not person_detected_arg and \
                     not combined_alarm_active: # Check combined_alarm_active global
                    fire_alarm_active_arg = True
                    if alarm_thread is None or not alarm_thread.is_alive():
                        stop_fire_alarm = False # Allow alarm to play
                        alarm_thread = threading.Thread(target=play_fire_alarm, daemon=True)
                        alarm_thread.start()
                        publish_alarm_status(True)

    if not fire_detected_this_frame:
        fire_start_time_arg = None # Reset timer for this specific call's context
        if fire_alarm_active_arg and not combined_alarm_active: # Check global
            stop_all_alarms() # This stops all pygame sounds
            fire_alarm_active_arg = False
            publish_alarm_status(False)
            
    return fire_detected_this_frame, fire_start_time_arg, fire_alarm_active_arg


# --- Flask Routes (Original for video) ---
@app.route('/')
def index():
    return render_template('index.html')

def generate_frames_for_web(): # Original name
    global latest_processed_frame, running # Use original global
    while running: # Use original global
        with frame_lock:
            if latest_processed_frame is None:
                time.sleep(0.1)
                continue
            frame_to_encode = latest_processed_frame.copy()

        (flag, encodedImage) = cv2.imencode(".jpg", frame_to_encode)
        if not flag:
            continue

        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + bytearray(encodedImage) + b'\r\n')
        time.sleep(0.03) 

@app.route('/video_feed')
def video_feed(): # Original name
    return Response(generate_frames_for_web(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

# --- ADDED: New API Endpoint for Dashboard Data ---
@app.route('/api/dashboard_data')
def api_dashboard_data_route(): # Added _route to avoid conflict if a var named api_dashboard_data exists
    with dashboard_data_lock:
        # Send copies to avoid issues if data is modified elsewhere while sending
        data_to_send = {
            "battery": latest_battery_info.copy(),
            "robot_status": latest_robot_status_info.copy()
        }
    return jsonify(data_to_send)
# ---

cap = None # Declare cap globally for cleanup, as in original signal_handler

def video_processing_thread_func(): # Original name
    global latest_processed_frame, fire_start_time, fire_alarm_active # Original globals
    global combined_start_time, combined_alarm_active, person_detected 
    global cap, running # Original globals
    
    cap = cv2.VideoCapture() # Original cap variable
    
    if not cap.isOpened():
        print("Error: Could not open video source: /dev/video2")
        running = False # Signal main thread to stop
        return

    try:
        while running: # Original global
            ret, frame = cap.read() # Original frame variable
            if not ret or frame is None: # Added check for None frame
                print("Failed to grab frame or frame is None")
                time.sleep(0.5)
                if not cap.isOpened(): # Check if cap got closed
                    rospy.logwarn("Camera /dev/video2 somehow closed. Attempting to reopen.")
                    cap.release()
                    cap = cv2.VideoCapture("/dev/video2")
                    if not cap.isOpened():
                        rospy.logerr("Failed to reopen camera. Video thread stopping.")
                        running = False # Stop if persistent failure
                        break # Exit loop
                continue

            frame = cv2.resize(frame, (frame_width, 360)) # Use global frame_width

            # --- Original Fire detection call ---
            # person_detected is a global that might be set by person detection logic later
            # For this call, pass the current global state of person_detected
            # The handle_fire_detection function returns fire_detected_this_frame, updated_fire_start_time, updated_fire_alarm_active
            current_frame_fire_detected, fire_start_time, fire_alarm_active = handle_fire_detection(
                frame, fire_start_time, fire_alarm_active, person_detected # Pass current person_detected state
            )
            # The globals fire_start_time and fire_alarm_active are updated by the return values

            # --- Person and Bottle detection (Original) ---
            person_results = person_model(frame, verbose=False) # Added verbose=False
            person_detected_in_this_frame = False # Local for this frame's processing
            bottle_detected_this_frame = False  # Local for this frame's processing
            bottle_distance = float('inf')

            if person_results[0].boxes is not None and len(person_results[0].boxes) > 0:
                boxes = person_results[0].boxes.xyxy.cpu().numpy().astype(int)
                class_ids = person_results[0].boxes.cls.cpu().numpy().astype(int)

                for box, class_id in zip(boxes, class_ids):
                    class_name = person_class_names[class_id].lower()
                    if class_name == "person":
                        person_detected_in_this_frame = True # Set local flag
                        x1, y1, x2, y2 = box
                        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 255), 2)
                        cv2.putText(frame, "Person", (x1, y1 - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
                    elif class_name == "bottle": # Original logic
                        bottle_detected_this_frame = True
                        x1, y1, x2, y2 = box
                        bottle_pixel_width = x2 - x1 # Renamed for clarity in this local scope
                        bottle_center_x_val = (x1 + x2) // 2 # Renamed
                        bottle_distance = estimate_distance(bottle_pixel_width)
                        
                        cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)
                        cv2.putText(frame, f"Bottle: {bottle_distance:.2f}m", (x1, y1 - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2) # Smaller font

                        if bottle_distance < 0.8:
                            obstacle_msg = Bool(); obstacle_msg.data = True
                            obstacle_detected_pub.publish(obstacle_msg)
                            side_msg = Float32(); side_msg.data = float(bottle_center_x_val - frame_center_x)
                            obstacle_side_pub.publish(side_msg)
                            cv2.putText(frame, "OBSTACLE DETECTED", (30, 130),
                                      cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2) # Smaller font

            # Update global person_detected state based on this frame
            person_detected = person_detected_in_this_frame

            if not bottle_detected_this_frame or bottle_distance >= 0.8: # Original logic
                obstacle_msg = Bool(); obstacle_msg.data = False
                obstacle_detected_pub.publish(obstacle_msg)

            # --- Combined Fire & Person detection (Original) ---
            if current_frame_fire_detected and person_detected: # Use current_frame_fire_detected
                if combined_start_time is None:
                    combined_start_time = time.time()
                    if fire_alarm_active: # If fire-only alarm was on
                        stop_all_alarms() # Stop it
                        fire_alarm_active = False # It's now a combined situation
                
                elif time.time() - combined_start_time >= combined_detection_threshold and \
                     not combined_alarm_active:
                    combined_alarm_active = True
                    play_emergency_alarm() # Plays once
                    publish_alarm_status(True) 
                
                cv2.putText(frame, "🔥 EMERGENCY: FIRE & PERSON DETECTED ⚠️", (10, 50), # Adjusted pos
                            cv2.FONT_HERSHEY_TRIPLEX, 0.8, (0, 0, 255), 2, cv2.LINE_AA) # Adjusted font/size
                
                if not combined_alarm_active and combined_start_time is not None : # Check timer started
                    detection_time = time.time() - combined_start_time
                    cv2.putText(frame, f"EVACUATE! Time: {detection_time:.1f}s", (10, 80), # Adjusted pos
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2) # Smaller
            else: # No combined fire and person
                if combined_alarm_active: # If combined alarm was active, turn it off
                    stop_all_alarms() 
                    combined_alarm_active = False
                    # Only set alarm status to False if fire-only alarm is also not active
                    if not fire_alarm_active:
                         publish_alarm_status(False)
                combined_start_time = None # Reset timer

            # Original display of fire detection time
            if current_frame_fire_detected and not combined_alarm_active and fire_start_time is not None:
                detection_time = time.time() - fire_start_time
                cv2.putText(frame, f"Fire detected for: {detection_time:.1f}s", (10, 25), # Adjusted pos
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2) # Smaller

            with frame_lock:
                latest_processed_frame = frame.copy()

            cv2.imshow("Detection System", frame) # Original window name
            if cv2.waitKey(1) & 0xFF == ord("q"):
                running = False # Original global
                break
    except KeyboardInterrupt:
        print("\nKeyboard interrupt received in video thread...")
        running = False # Original global
    except Exception as e:
        print(f"\nError occurred in video thread: {str(e)}")
        running = False # Original global
    finally:
        stop_all_alarms() # Original func
        if cap is not None and cap.isOpened(): # Check cap, not video_capture_device
            cap.release()
        cv2.destroyWindow("Detection System") # Close specific window
        # cv2.destroyAllWindows() # Original was destroyAllWindows
        print("Video processing thread finished.")
        with frame_lock:
            latest_processed_frame = None


if __name__ == '__main__':
    video_thread_instance = None # Keep original-like name if any
    try:
        print("Starting video processing thread...")
        video_thread_instance = threading.Thread(target=video_processing_thread_func, daemon=True)
        video_thread_instance.start()

        print("Starting Flask web server on http://0.0.0.0:5000")
        print("Access the stream from another device on the same network at http://<your_laptop_ip>:5000")
        
        # Original Flask run
        app.run(host='0.0.0.0', port=5000, debug=False, threaded=True, use_reloader=False)
        
    except KeyboardInterrupt:
        print("\nFlask server (main thread) shutting down...")
        running = False # Signal other threads
    except SystemExit: # From signal_handler
        print("\nFlask server (main thread) received SystemExit...")
        running = False # Signal other threads
    except Exception as e:
        print(f"\nError occurred in main: {str(e)}")
        running = False # Signal other threads
    finally:
        # running = False # Ensure it's set here too for all exit paths
        stop_all_alarms() # Original func
        pygame.mixer.quit() # Original
        
        if video_thread_instance is not None and video_thread_instance.is_alive(): # Original check
            print("Waiting for video processing thread to finish...")
            video_thread_instance.join(timeout=5.0)
            if video_thread_instance.is_alive():
                 print("Video processing thread did not finish in time.")
        
        print("Application finished.")