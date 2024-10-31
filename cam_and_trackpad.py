"""

This code synchronizes camera input and trackpad input to generate a synchronized output.

"""


import time
import threading
from collections import deque
from datetime import datetime
import pyautogui
import cv2

from utils import lcm_float, gcd_float

"""

CONFIG

"""
# maximum lengths for the data queues to prevent unlimited growth
TRACKPAD_QUEUE_MAXLEN = 1000
CAMERA_QUEUE_MAXLEN = 1000

# how often trackpad and camera send values (in seconds)
TRACKPAD_SAMPLING_INTERVAL = 1
CAMERA_SAMPLING_INTERVAL = 0.5

USE_LCM = True

# how often the sensors are sampled
if USE_LCM:
    SYNC_SAMPLING_INTERVAL = lcm_float(CAMERA_SAMPLING_INTERVAL, TRACKPAD_SAMPLING_INTERVAL)
else:
    SYNC_SAMPLING_INTERVAL = gcd_float(CAMERA_SAMPLING_INTERVAL, TRACKPAD_SAMPLING_INTERVAL)

# represents each stream of data as a deque (older values are earlier in the deque)
# fixed length to prevent too many values from being read. When length is reached, oldest values discarded automatically!
trackpad_stream = deque(maxlen=TRACKPAD_QUEUE_MAXLEN)
camera_stream = deque(maxlen=CAMERA_QUEUE_MAXLEN)

# locks for each of the streams 
# personal Note: since Python has GIL, that makes individual append/length check operations for the deques thread safe (which I do in synchronize_streams)
# however, doing a sequence of these operations are not automatically thread safe. so we need locks. 
trackpad_lock = threading.Lock()
camera_lock = threading.Lock()

def capture_trackpad():
    """
    captures the current position of the mouse cursor at some defined interval & stores in the trackpad_stream queue. 
    """
    while True:
        timestamp = datetime.now()
        position = pyautogui.position() 

        # Add trackpad position to the queue
        with trackpad_lock:
            trackpad_stream.append((timestamp, position))
        time.sleep(TRACKPAD_SAMPLING_INTERVAL)  # Wait for the next interval

def capture_camera():
    """
    captures frames from the default camera at some defined interval & stores in the camera_stream queue.
    """
    cap = cv2.VideoCapture(0)  # Open default camera
    if not cap.isOpened():
        print("error: couldn't open camera.")
        return
    
    print("camera successfully opened.")

    while True:
        ret, frame = cap.read()
        if ret:
            timestamp = datetime.now()
            
            # Convert frame to JPEG
            frame_data = cv2.imencode('.jpg', frame)[1]
            with camera_lock:
                camera_stream.append((timestamp, frame_data))
        else:
            print("couldn't capture frame.")
            break

        time.sleep(CAMERA_SAMPLING_INTERVAL)  # sleep until next interval is reached


def synchronize_streams():
    """
    samples the trackpad and camera queues at SYNC_SAMPLING_INTERVAL.
    """
    print("started reading data.")

    with open('trackpad_cam_data.txt', 'w') as f:
        while True:
            sample_time = datetime.now()
            data_tuple = [sample_time.strftime('%H:%M:%S.%f')] 

            # read newest trackpad position from stream
            with trackpad_lock:
                if trackpad_stream:
                    latest_trackpad = trackpad_stream[-1]
                    data_tuple.append(str(latest_trackpad[1]))  
                else:
                    data_tuple.append("no_trackpad_data")

            # retrieve the latest camera frame
            frame_available = False
            frame = None

            # read latest camera frame from stream
            with camera_lock:
                if camera_stream:
                    latest_camera = camera_stream[-1]
                    frame_available = True

                    # decode the latest frame from JPEG format
                    frame = cv2.imdecode(latest_camera[1], cv2.IMREAD_COLOR)
                else:
                    frame_available = False

            if not frame_available:
                print("camera frame not available. Waiting for next synchronization interval.")
                data_tuple.append("camera_not_found")
            else:
                data_tuple.append("camera_found")

            # write synchronized data
            f.write(f"timestamp: {data_tuple[0]}, cursor: {data_tuple[1]}, camera: {data_tuple[2]}\n")

            if frame_available and frame is not None:
                cv2.imshow("Camera Frame", frame)

                # Quit logic
                if cv2.waitKey(1) & 0xFF == ord('q'):  # use mask to remove excess bits
                    print("Exiting synchronization loop...")
                    break

            # sleep main threaad until next synchronization interval
            time.sleep(SYNC_SAMPLING_INTERVAL)


# spawn two threads: one for the trackpad capture function and another for the camera capture function.
# note: I made these daemon threads to prevent them from blocking the main thread (they're executing in the background).
trackpad_thread = threading.Thread(target=capture_trackpad, daemon=True)
camera_thread = threading.Thread(target=capture_camera, daemon=True)

trackpad_thread.start()
camera_thread.start()

if __name__ == "__main__":
    try:
        synchronize_streams()
    except KeyboardInterrupt:
        print("Program complete.")
    finally:
        cv2.destroyAllWindows()
