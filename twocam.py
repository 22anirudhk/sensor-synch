"""

This code meshes two camera inputs to generate a directory synchronized frames in sorted order.

"""

import time
import threading
from collections import deque
from datetime import datetime
import cv2
import os
import shutil  
from utils import lcm_float, gcd_float

"""
CONFIG
"""

# true if you want to clean the output directory on startup
RESET_OUTPUT_DIR_ON_START = True

# directory to save synchronized data and frames
OUTPUT_DIR = "synchronized_output"
CAMERA_FRAMES_DIR = os.path.join(OUTPUT_DIR, "camera_frames")


# how often cameras send values (in seconds)
# reciprocal is frequency in hz of how many frames being sent per second
CAM_0_INTERVAL = 1.0
CAM_1_INTERVAL = 0.25

# set this to the LCM of the sampling intervals
USE_LCM = True
if USE_LCM:
    SYNC_SAMPLING_INTERVAL = lcm_float(CAM_0_INTERVAL, CAM_1_INTERVAL)
else:
    SYNC_SAMPLING_INTERVAL = gcd_float(CAM_0_INTERVAL, CAM_1_INTERVAL)

print(f"SYNC_SAMPLING_INTERVAL: {SYNC_SAMPLING_INTERVAL}")

# maximum lengths for the camera data queues to prevent unlimited growth
CAMERA_QUEUE_MAXLEN = 1000

# represents each camera stream of data as a deque (older values are earlier in the deque)
camera0_stream = deque(maxlen=CAMERA_QUEUE_MAXLEN)
camera1_stream = deque(maxlen=CAMERA_QUEUE_MAXLEN)

# locks for each deque
camera0_lock = threading.Lock()
camera1_lock = threading.Lock()

def setup_output_directory():
    if RESET_OUTPUT_DIR_ON_START:
        try:
            if os.path.exists(OUTPUT_DIR):
                shutil.rmtree(OUTPUT_DIR)
            os.makedirs(CAMERA_FRAMES_DIR, exist_ok=True)
        except Exception as e:
            print(f"error resetting output directory: {e}")
            raise
    else:
        # make sure the directories exist
        os.makedirs(CAMERA_FRAMES_DIR, exist_ok=True)


def capture_camera(camera_index, camera_stream, camera_lock, interval):
    """
    captures frames from the specified camera at defined intervals & stores in the provided camera_stream queue.
    """
    cap = cv2.VideoCapture(camera_index) 
    if not cap.isOpened():
        print(f"Error: couldn't open camera {camera_index}.")
        return
    
    print(f"Camera {camera_index} successfully opened.")

    while True:
        ret, frame = cap.read()
        if ret:
            # convert frame to JPEG
            frame_data = cv2.imencode('.jpg', frame)[1]
            with camera_lock:
                camera_stream.append(frame_data)
        else:
            print(f"Couldn't capture frame from camera {camera_index}.")
            break

        time.sleep(interval)

def synchronize_streams():
    """
    samples the latest data from both camera streams at defined synchronization intervals and processes or saves them. 
    runs on the main thread.
    """
    print("Started reading data.")

    synchronized_data_file = os.path.join(OUTPUT_DIR, 'synchronized_data.csv')
    try:
        with open(synchronized_data_file, 'w') as f:
            f.write("synch_timestamp, path_to_cam_0_frame, path_to_cam_1_frame\n")
            
            while True:
                # get the current synchronization time
                sample_time = datetime.now()
                timestamp_str = sample_time.strftime('%Y-%m-%d_%H-%M-%S-%f')  # For filenames

                # initialize output data with the timestamp
                data_tuple = [sample_time.strftime('%Y-%m-%d %H:%M:%S.%f')]

                # retrieve the latest camera0 frame
                frame0_path = ""
                frame0 = None

                # retrieve the latest camera0 frame
                with camera0_lock:
                    if camera0_stream:
                        latest_camera0 = camera0_stream[-1]
                        frame0 = cv2.imdecode(latest_camera0, cv2.IMREAD_COLOR)

                        # save frame if available
                        if frame0 is not None:
                            frame0_filename = os.path.join(CAMERA_FRAMES_DIR, f"{timestamp_str}_camera0.jpg")
                            cv2.imwrite(frame0_filename, frame0)
                            frame0_path = frame0_filename

                frame1_path = ""
                frame1 = None

                # retrieve the latest camera1 frame
                with camera1_lock:
                    if camera1_stream:
                        latest_camera1 = camera1_stream[-1]
                        frame1 = cv2.imdecode(latest_camera1, cv2.IMREAD_COLOR)

                        if frame1 is not None:
                            frame1_filename = os.path.join(CAMERA_FRAMES_DIR, f"{timestamp_str}_camera1.jpg")
                            cv2.imwrite(frame1_filename, frame1)
                            frame1_path = frame1_filename

                # append frame paths to data_tuple
                data_tuple.append(frame0_path if frame0_path else "No frame captured")
                data_tuple.append(frame1_path if frame1_path else "No frame captured")

                # write synchronized data
                f.write(', '.join(data_tuple) + '\n')
                f.flush() 

                if frame0 is not None:
                    cv2.imshow("Camera 0 Frame", frame0)
                if frame1 is not None:
                    cv2.imshow("Camera 1 Frame", frame1)

                # check for quit
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    print("Exiting synchronization loop...")
                    break

                time.sleep(SYNC_SAMPLING_INTERVAL)
    except Exception as e:
        print(f"Error during synchronization: {e}")
    finally:
        cv2.destroyAllWindows()

if __name__ == "__main__":
    setup_output_directory()

    # spawn threads for both cameras
    # note: set as daemon thread so main thread isn't blocked
    camera0_thread = threading.Thread(target=capture_camera, args=(0, camera0_stream, camera0_lock, CAM_0_INTERVAL), daemon=True)
    camera1_thread = threading.Thread(target=capture_camera, args=(1, camera1_stream, camera1_lock, CAM_1_INTERVAL), daemon=True)
    
    camera0_thread.start()
    camera1_thread.start()

    try:
        synchronize_streams()
    except KeyboardInterrupt:
        print("Program interrupted by user.")
    finally:
        cv2.destroyAllWindows()
