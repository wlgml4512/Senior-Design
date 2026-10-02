"""Live ArUco-based target tracking process for ATLAS.

This script owns the camera loop, target detection, and spatial estimation for
the vision subsystem, and it publishes the robot's tracking state by writing
structured log lines that fusion later tails.
"""

import cv2
import time
from pathlib import Path
from picamera2 import Picamera2
from detector import ArucoTargetDetector
from estimator import SpatialEstimator
from tracker import TargetTracker
import logging

def run_vision():
    log_path = Path(__file__).with_name("log.txt")
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(
        {"size": (320, 320), "format": "BGR888"}, 
        controls={"FrameRate": 60}
    )
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s [%(levelname)s] %(message)s',
        handlers=[
            logging.FileHandler(log_path),
            logging.StreamHandler()
        ],
        force=True,
    )
    picam2.configure(config)
    picam2.start()

    detector = ArucoTargetDetector(target_id=5)
    tracker = TargetTracker(max_lost_time=5.0)
    estimator = SpatialEstimator(frame_width=320, marker_size_m=0.09)

    prev_frame_time = 0
    new_frame_time = 0

    try:
        while True:
            start_time = time.perf_counter()
            frame = picam2.capture_array()

            new_frame_time = time.time()
            fps = 1 / (new_frame_time - prev_frame_time)
            prev_frame_time = new_frame_time
            target_info = detector.detect(frame)
            tracked_state = tracker.update_and_lock(target_info)

            if isinstance(tracked_state, dict):
                target_angle = estimator.estimate_angle(tracked_state['cx'])
                target_distance = estimator.estimate_distance(tracked_state['w'])

                # print(f"ID: {tracked_state['id']} | "
                #       f"Angle: {target_angle:+.2f} deg | "
                #       f"Dist: {target_distance:.2f} m | Conf: 1.00")
                msg = f"ID: {tracked_state['id']} | Angle: {target_angle:+.2f} deg | Dist: {target_distance:.2f} m"
                logging.info(msg)
                cv2.aruco.drawDetectedMarkers(frame, [tracked_state['corners']])

            elif tracked_state == "wait_state":
                # print("waiting for target")
                logging.warning("waiting for target")
                cv2.putText(frame, "waiting", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            else:
                # print("lost target")
                logging.error("lost target")
            cv2.putText(frame, f"FPS: {int(fps)}", (7, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (100, 255, 0), 2, cv2.LINE_AA)
            #display_frame = cv2.resize(frame, (640, 640), interpolation=cv2.INTER_NEAREST)
            #cv2.imshow("demo showing", display_frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("shutdown")
                break

            elapsed_time = (time.perf_counter() - start_time) * 1000.0
            if elapsed_time < 16.0:
                time.sleep((16.0 - elapsed_time) / 1000.0)

    except KeyboardInterrupt:
        print("\nshutdown")
    finally:
        picam2.stop()
        picam2.close()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    run_vision()
