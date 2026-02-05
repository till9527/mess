# === controller_qcar.py (Refactored Brain - No QLabs) ===
import multiprocessing
import time
import math
import numpy as np
from multiprocessing.managers import DictProxy
from threading import Thread
from pal.products.qcar import IS_PHYSICAL_QCAR  # Still useful to know
import queue

MAP_SCALER_X = 0.03935
MAP_SCALER_Y = -0.03607  # Note: Y-axis is inverted
MAP_OFFSET_X = 1.3607
MAP_OFFSET_Y = 2.9348
# --- END NEW ---

geofencing_threshold = 1.2
# --- REMOVED: QLabs Imports ---

# --- Perception Thresholds ---
RED_LIGHT_MIN_WIDTH = 30
RED_LIGHT_MIN_HEIGHT = 30
MOVEMENT_THRESHOLD_PX_PER_SEC = 25
STOP_SIGN_MIN_WIDTH = 50
STOP_SIGN_WAIT_TIME_S = 5.0
DANGER_ZONE_LENGTH = 3.0
DANGER_ZONE_WIDTH = 2.0
PIXELS_PER_METER = 60
STALE_OBJECT_TIMEOUT = 1.5
QCAR_DANGER_WIDTH = 140
CAMERA_CENTER_X = 320
CENTER_TOLERANCE = 150
PEDESTRIAN_MIN_WIDTH_FOR_STOP = 40
PEDESTRIAN_MIN_HEIGHT_FOR_STOP = 150
QCAR_MIN_WIDTH_FOR_STOP = 120
SAFE_FOLLOWING_DISTANCE_WIDTH = 80
MIN_DISTANCE_FOR_HARD_BRAKE_WIDTH = 150
DISTANCE_TOLERANCE_WIDTH = 10
LEAD_CAR_CRAWL_SPEED_THRESHOLD = 5.0
ACC_CYCLE_DURATION = 0.5
MAX_SPEED_PXS = 150.0
PEDESTRIAN_CLEAR_TIMEOUT_S = 2.5


# --- V2X Configuration (for Geofencing) ---
# We still need the LOCATIONS of the lights
# ** IMPORTANT: IDs match environment_logic.py (4 and 3) **
# This list is used to map the statuses received from the perception module
# (which should be in the order [ID 4, ID 3]) to the correct geofence area.


# --- Helper functions ---
def get_position(results):
    if results and "x" in results[0] and "y" in results[0] and "width" in results[0]:
        x = results[0]["x"]
        width = results[0]["width"]
        height = results[0].get("height", width)
        center_x = x + width / 2
        center_y = results[0]["y"] + height / 2
        return (center_x, center_y)
    return None


def any_detected_objects(results):
    return isinstance(results, list) and len(results) > 0


def get_cls(results):
    return results[0]["class"] if results and "class" in results[0] else None


def get_width(results):
    return results[0]["width"] if results and "width" in results[0] else 0


def get_x(results):
    return results[0]["x"] if results and "x" in results[0] else 0


def get_y(results):
    return results[0]["y"] if results and "y" in results[0] else 0


def get_height(results):
    return results[0]["height"] if results and "width" in results[0] else 0


# --- MODIFIED: Main function signature (simplified) ---
def main(
    perception_queue: multiprocessing.Queue,
    command_queue: multiprocessing.Queue,
    shared_pose: DictProxy,
):

    # --- V2X State Variables (Local) ---
    is_stopped_v2x_light = False
    geofencing_areas = []
    has_stopped_at = {}

    # --- Perception State Variables ---
    is_stopped_light = False  # For perception-based red light
    is_stopped_pedestrian = False
    tracked_objects = {}
    is_moving_ped = False
    is_stopped_yield_sign = False
    is_stopped_for_sign = False
    is_stopped_qcar_red_light = False
    stop_sign_start_time = 0
    yield_sign_sign_start_time = 0
    last_green_light_seen_time = 0
    red_light_start_time = 0
    is_stopped_qcar = False
    last_stop_qcar = 0
    last_pedestrian_seen_time = 0
    last_stop_seen_time = 0
    last_yield_sign_seen_time = 0
    last_red_light_seen_time = 0
    last_qcar_seen_time = 0
    last_light_seen_time = 0

    # --- Overall Command State ---
    last_command_was_stop = False

    # --- NEW: Timer for printing V2X status ---
    last_v2x_print_time = 0.0

    try:
        # --- Main Control Loop (Continuous) ---
        while True:
            current_time = time.time()

            # --- 1. GET BUNDLED DATA from Perception Module ---
            results = []  # <-- FIX 1: Clear *only* perception results

            if not perception_queue.empty():
                input_data = perception_queue.get()
                results = input_data.get("detections", [])
                # <-- FIX 1: *Only* update V2X status when new data arrives.
                # It is no longer reset to [] every loop.

            # --- 3. PERCEPTION LOGIC (uses 'results' from queue) ---
            stale_keys = []
            for key, data in tracked_objects.items():
                if current_time - data["time"] > STALE_OBJECT_TIMEOUT:
                    stale_keys.append(key)
            for key in stale_keys:
                del tracked_objects[key]

            if any_detected_objects(results):
                for i, det in enumerate(results):
                    # Check for QCar with your specific priority height (e.g., > 70 or > 100)
                    if det["class"] == "Qcar" and det.get("height", 0) > 125:
                        # Move this detection to the top of the list (Index 0)
                        results.insert(0, results.pop(i))
                        break
                cls = get_cls(results)
                width = get_width(results)
                height = get_height(results)
                position = get_position(results)
                x = get_x(results)
                is_moving_ped = False

                # Update timestamps
                if cls == "Qcar":
                    last_qcar_seen_time = current_time
                # print(height)
                # print("Qcar width is: ", width)
                # print("Qcar height is: ", height)
                # print(
                #     "These many seconds have passed since we saw a red light: ",
                #     current_time - last_red_light_seen_time,
                # )
                if (cls == "red_light" or cls == "green_light") and (
                    width > RED_LIGHT_MIN_WIDTH and height > RED_LIGHT_MIN_HEIGHT
                ):
                    last_light_seen_time = current_time
                if cls == "pedestrian":
                    last_pedestrian_seen_time = current_time
                if cls == "stop_sign":
                    last_stop_seen_time = current_time
                if cls == "yield_sign":
                    last_yield_sign_seen_time = current_time
                if cls == "red_light":
                    last_red_light_seen_time = current_time
                if cls == "green_light":
                    last_green_light_seen_time = current_time

                if cls == "yellow_light":
                    last_light_seen_time = current_time

                # print("last light seen time was: ", last_light_seen_time)
                # print("current time is: ", current_time)
                if position and cls in tracked_objects:
                    last_pos = tracked_objects[cls]["position"]
                    last_time = tracked_objects[cls]["time"]
                    delta_time = current_time - last_time
                    if delta_time > 0:
                        distance = math.hypot(
                            position[0] - last_pos[0], position[1] - last_pos[1]
                        )
                        speed_px_per_sec = distance / delta_time
                        if cls == "pedestrian" and (
                            speed_px_per_sec > MOVEMENT_THRESHOLD_PX_PER_SEC
                        ):
                            is_moving_ped = True
                if position:
                    tracked_objects[cls] = {
                        "position": position,
                        "time": current_time,
                    }

                # --- Perception Stop Conditions ---
                if (
                    cls == "red_light"
                    and width > RED_LIGHT_MIN_WIDTH
                    and height > RED_LIGHT_MIN_HEIGHT
                    and not is_stopped_for_sign
                    and not is_stopped_light
                    and not is_stopped_qcar
                    and not is_stopped_qcar_red_light
                    and not is_stopped_yield_sign
                    and not is_stopped_v2x_light
                    and not is_stopped_pedestrian
                ):
                    is_stopped_light = True
                    red_light_start_time = current_time

                elif (
                    cls == "green_light"
                    and width > RED_LIGHT_MIN_WIDTH
                    and height > RED_LIGHT_MIN_HEIGHT
                    and not is_stopped_for_sign
                    and is_stopped_light
                    and not is_stopped_v2x_light
                    and not is_stopped_qcar
                    and not is_stopped_qcar_red_light
                    and not is_stopped_yield_sign
                    and not is_stopped_pedestrian
                ):
                    is_stopped_light = False

                elif cls == "Qcar" and height > 125:  # Simplified QCar following
                    is_stopped_qcar = True
                    last_stop_qcar = current_time
                    # print("stopped because qcar too close - height")

                elif (
                    cls == "Qcar"
                    and is_stopped_qcar
                    and not is_stopped_light
                    and not is_stopped_v2x_light
                    and not is_stopped_qcar_red_light
                    and not is_stopped_yield_sign
                    and not is_stopped_pedestrian
                    and height <= 125
                ):
                    is_stopped_qcar = False
                    # print("Resumed because qcar far enough - height")

                elif (
                    cls == "stop_sign"
                    and not is_stopped_for_sign  # Only trigger once
                    and width > STOP_SIGN_MIN_WIDTH
                    and current_time - stop_sign_start_time > 10  # Cooldown
                    and not is_stopped_light
                    and not is_stopped_v2x_light
                    and not is_stopped_qcar
                    and not is_stopped_qcar_red_light
                    and not is_stopped_yield_sign
                    and not is_stopped_pedestrian
                ):
                    is_stopped_for_sign = True
                    stop_sign_start_time = current_time

                elif (
                    cls == "yield_sign"
                    and not is_stopped_yield_sign  # Only trigger once
                    and not is_stopped_light
                    and not is_stopped_v2x_light
                    and not is_stopped_qcar
                    and not is_stopped_qcar_red_light
                    and not is_stopped_for_sign
                    and not is_stopped_pedestrian
                    and width > 30
                    and current_time - yield_sign_sign_start_time > 6
                ):
                    is_stopped_yield_sign = True
                    yield_sign_sign_start_time = current_time

                elif (
                    cls == "pedestrian"
                    and (not 100 < x < 450 or width < PEDESTRIAN_MIN_WIDTH_FOR_STOP)
                    and is_stopped_pedestrian
                    and not is_stopped_yield_sign  # Only trigger once
                    and not is_stopped_light
                    and not is_stopped_v2x_light
                    and not is_stopped_qcar
                    and not is_stopped_qcar_red_light
                    and not is_stopped_for_sign
                ):
                    is_stopped_pedestrian = False
                elif (
                    cls == "pedestrian"
                    and width > PEDESTRIAN_MIN_WIDTH_FOR_STOP
                    and 100 < x < 450
                    and not is_stopped_yield_sign  # Only trigger once
                    and not is_stopped_light
                    and not is_stopped_v2x_light
                    and not is_stopped_qcar
                    and not is_stopped_qcar_red_light
                    and not is_stopped_for_sign
                    and not is_stopped_pedestrian
                ):
                    is_stopped_pedestrian = True

            # --- 4. TIMEOUT LOGIC ---
            if is_stopped_pedestrian and (
                current_time - last_pedestrian_seen_time > PEDESTRIAN_CLEAR_TIMEOUT_S
            ):
                is_stopped_pedestrian = False

            if is_stopped_qcar and (current_time - last_qcar_seen_time > 1):
                is_stopped_qcar = False
                print("no longer stopped for qcar - height")

            if is_stopped_for_sign and (
                current_time - stop_sign_start_time > STOP_SIGN_WAIT_TIME_S
            ):
                is_stopped_for_sign = False

            if is_stopped_yield_sign and (
                current_time - yield_sign_sign_start_time > 3
            ):
                is_stopped_yield_sign = False
            if is_stopped_light and (current_time - last_light_seen_time > 3):
                is_stopped_light = False

            # --- 5. *** FIX 2: FINAL DECISION BLOCK (with V2X Priority) *** ---

            should_stop = False

            # Priority 1: V2X Rules (as you requested)

            all_perception_conditions = {
                "Perception_Light": is_stopped_light,
                "Pedestrian": is_stopped_pedestrian,
                "Stop_Sign": is_stopped_for_sign,
                "Yield_Sign": is_stopped_yield_sign,
                "QCar_Too_Close": is_stopped_qcar,
                "QCar_Too_Close_Light": is_stopped_qcar_red_light,
                "V2X_Light": is_stopped_v2x_light,
            }

            # Calculate current reasons immediately
            current_reasons = [k for k, v in all_perception_conditions.items() if v]

            if current_reasons:
                should_stop = True
                # Update the persistent tracker while we are stopped
                active_stop_reasons = current_reasons
            else:
                should_stop = False

            # Now, send the command based on the final decision
            if should_stop and not last_command_was_stop:
                command_queue.put("STOP")
                last_command_was_stop = True
                print(f"[Controller] STOPPING: Reasons: {active_stop_reasons}")

            elif not should_stop and last_command_was_stop:
                command_queue.put("GO")
                last_command_was_stop = False

                # We use active_stop_reasons here, which holds the data from the last frame
                # where we were still stopped.
                print(
                    f"[Controller] RESUMING: Condition(s) cleared: {active_stop_reasons}"
                )

                # Optional: clear the tracker after resuming
                active_stop_reasons = []

            # --- 6. LOOP DELAY ---
            time.sleep(0.05)  # Poll V2X and perception at 20Hz

    except KeyboardInterrupt:
        print("[Controller] Shutdown requested.")
    finally:
        # --- REMOVED: QLabs Cleanup ---
        print("[Controller] Process terminated.")
