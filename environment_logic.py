import sys
import time
import math
from threading import Thread
import threading
# Quanser Imports
from qvl.qlabs import QuanserInteractiveLabs
from qvl.real_time import QLabsRealTime
from qvl.free_camera import QLabsFreeCamera
from qvl.crosswalk import QLabsCrosswalk
from qvl.traffic_light import QLabsTrafficLight
from qvl.person import QLabsPerson
from qvl.stop_sign import QLabsStopSign
from qvl.yield_sign import QLabsYieldSign
from qvl.environment_outdoors import QLabsEnvironmentOutdoors
from qvl.system import QLabsSystem
from qvl.walls import QLabsWalls
from qvl.qcar_flooring import QLabsQCarFlooring
from qvl.roundabout_sign import QLabsRoundaboutSign
from qvl.basic_shape import QLabsBasicShape

# --- Configuration Constants ---
TRAFFIC_LIGHTS_CONFIG = [
    {"id": 4, "location": [23.667, 9.893, 0.005], "rotation": [0, 0, 0]},
    {"id": 3, "location": [-21.122, 9.341, 0.005], "rotation": [0, 0, 180]},
]
STOP_SIGNS_CONFIG = [
    {"id": 100, "location": [-6.238, 6.47, 0.2], "rotation": [0, 0, 180]},
    {"id": 101, "location": [-2.067, 16.986, 0.215], "rotation": [0, 0, 90]},
    {"id": 102, "location": [7.989, 13.371, 0.215], "rotation": [0, 0, 360]},
    {"id": 103, "location": [4.733, 2.166, 0.215], "rotation": [0, 0, 270]},
    # {"id": 104, "location": [16.412, -13.551, 0.2], "rotation": [0, 0, 180]},
]
YIELD_SIGNS_CONFIG = [
    {"id": 200, "location": [25.007, 32.494, 0.2], "rotation": [0, 0, -90]},
    {"id": 201, "location": [5.25, 39.477, 0.215], "rotation": [0, 0, 180]},
    {"id": 202, "location": [11.136, 28.326, 0.215], "rotation": [0, 0, 225]},
]

CROSSWALK_START = [17.584, 18.098, 0.215]
CROSSWALK_END = [24.771, 18.023, 0.19]
PEDESTRIAN_ROTATION = [0, 0, math.pi / 2]
CROSSWALK_LOCATION = [21.175, 18.15, 0.0]


# (Other constants)
def setup_node_following_map(qlabs):
    """
    Reconstructs the specific walls, flooring, and signs
    from node_following_only.py
    """
    # 1. Flooring
    x_offset = 1.3
    y_offset = 16.7
    hFloor = QLabsQCarFlooring(qlabs)
    hFloor.spawn_degrees(
        [x_offset, y_offset, 0.01], rotation=[0, 0, -90], scale=[10, 10, 10]
    )

    # 2. Walls
    hWall = QLabsWalls(qlabs)
    hWall.set_enable_dynamics(False)

    # Vertical Walls (Left/Right)
    for y in range(5):
        hWall.spawn_degrees(
            location=[-24.0 + x_offset, (-y * 10.0) + 25.5 + y_offset, 0.01],
            rotation=[0, 0, 0],
            scale=[10, 10, 10],
        )
    for y in range(6):
        hWall.spawn_degrees(
            location=[24.0 + x_offset, (-y * 10.0) + 25.5 + y_offset, 0.01],
            rotation=[0, 0, 0],
            scale=[10, 10, 10],
        )

    # Horizontal Walls (Top/Bottom)
    for x in range(5):
        hWall.spawn_degrees(
            location=[-19.0 + x * 10.0 + x_offset, 30.5 + y_offset, 0.01],
            rotation=[0, 0, 90],
            scale=[10, 10, 10],
        )
    for x in range(4):
        hWall.spawn_degrees(
            location=[-9.0 + x * 10.0 + x_offset, -30.5 + y_offset, 0.01],
            rotation=[0, 0, 90],
            scale=[10, 10, 10],
        )

    # Angled Walls
    hWall.spawn_degrees(
        location=[-20.3 + x_offset, -22.75 + y_offset, 0.01],
        rotation=[0, 0, 48],
        scale=[10, 10, 10],
    )
    hWall.spawn_degrees(
        location=[-15.75 + x_offset, -27.0 + y_offset, 0.01],
        rotation=[0, 0, 48],
        scale=[10, 10, 10],
    )

    # 3. Static Signs (Stop, Roundabout, Yield)
    # Note: We spawn these here directly instead of using the config lists
    # to match the node_following map exactly.

    # Stop Signs
    stop = QLabsStopSign(qlabs)
    stop.spawn_degrees(
        [-15.0, 36.0, 0.06], [0, 0, -35], [1, 1, 1], waitForConfirmation=False
    )
    stop.spawn_degrees(
        [-15.0, 22.0, 0.06], [0, 0, 35], [1, 1, 1], waitForConfirmation=False
    )
    stop.spawn_degrees(
        [24.10, 2.06, 0.06], [0, 0, -90], [1, 1, 1], waitForConfirmation=False
    )
    stop.spawn_degrees(
        [17.66, 16.97, 0.06], [0, 0, 90], [1, 1, 1], waitForConfirmation=False
    )

    # Roundabout Signs
    rb = QLabsRoundaboutSign(qlabs)
    rb.spawn_degrees(
        [23.92, 25.22, 0.06], [0, 0, -90], [1, 1, 1], waitForConfirmation=False
    )
    rb.spawn_degrees(
        [6.98, 24.83, 0.06], [0, 0, -145], [1, 1, 1], waitForConfirmation=False
    )
    rb.spawn_degrees(
        [0.07, 39.73, 0.06], [0, 0, 135], [1, 1, 1], waitForConfirmation=False
    )

    # Yield Signs
    yield_s = QLabsYieldSign(qlabs)
    yield_s.spawn_degrees(
        [0.0, -13.0, 0.06], [0, 0, -180], [1, 1, 1], waitForConfirmation=False
    )
    yield_s.spawn_degrees(
        [24.0, 32.0, 0.06], [0, 0, -90], [1, 1, 1], waitForConfirmation=False
    )
    yield_s.spawn_degrees(
        [11.0, 28.0, 0.06], [0, 0, -145], [1, 1, 1], waitForConfirmation=False
    )
    yield_s.spawn_degrees(
        [4.9, 38.0, 0.06], [0, 0, 135], [1, 1, 1], waitForConfirmation=False
    )

    # 4. Crosswalks
    cw = QLabsCrosswalk(qlabs)
    cw.spawn_degrees([-18.7, 1.95, 0.1], [0, 0, 0], [1, 1, 0.75], configuration=0)
    cw.spawn_degrees([-5.0, 9.5, 0.06], [0, 0, 90], [1, 1, 0.75], configuration=0)
    cw.spawn_degrees([1.5, 3.2, 0.06], [0, 0, 0], [1, 1, 0.75], configuration=0)
    cw.spawn_degrees([7.5, 9.5, 0.06], [0, 0, 90], [1, 1, 0.75], configuration=0)
    cw.spawn_degrees([1.3, 15.7, 0.06], [0, 0, 0], [1, 1, 0.75], configuration=0)
    cw.spawn_degrees([14.5, 9.5, 0.06], [0, 0, 90], [1, 1, 0.75], configuration=0)

    # 5. Splines (Yellow Lines)
    sp = QLabsBasicShape(qlabs)
    sp.spawn_degrees(
        [22.1, 2.0, 0.06], [0, 0, 0], [2.7, 0.2, 0.01], waitForConfirmation=False
    )
    sp.spawn_degrees(
        [19.51, 16.8, 0.06], [0, 0, 0], [2.7, 0.2, 0.01], waitForConfirmation=False
    )
    sp.spawn_degrees(
        [-0.5, -10.2, 0.06], [0, 0, 90], [3.8, 0.2, 0.01], waitForConfirmation=False
    )

    print("Map elements spawned successfully.")


def traffic_light_logic(traffic_light_1, traffic_light_2, traffic_light_3, traffic_light_4, interval_s=5):
    """Run a 4-way traffic light cycle matching Setup_Real_Scenario.py."""
    intersection_flag = 0
    while True:
        try:
            if intersection_flag == 0:
                traffic_light_1.set_color(color=QLabsTrafficLight.COLOR_RED)
                traffic_light_3.set_color(color=QLabsTrafficLight.COLOR_RED)
                traffic_light_2.set_color(color=QLabsTrafficLight.COLOR_GREEN)
                traffic_light_4.set_color(color=QLabsTrafficLight.COLOR_GREEN)
            elif intersection_flag == 1:
                traffic_light_1.set_color(color=QLabsTrafficLight.COLOR_RED)
                traffic_light_3.set_color(color=QLabsTrafficLight.COLOR_RED)
                traffic_light_2.set_color(color=QLabsTrafficLight.COLOR_YELLOW)
                traffic_light_4.set_color(color=QLabsTrafficLight.COLOR_YELLOW)
            elif intersection_flag == 2:
                traffic_light_1.set_color(color=QLabsTrafficLight.COLOR_GREEN)
                traffic_light_3.set_color(color=QLabsTrafficLight.COLOR_GREEN)
                traffic_light_2.set_color(color=QLabsTrafficLight.COLOR_RED)
                traffic_light_4.set_color(color=QLabsTrafficLight.COLOR_RED)
            elif intersection_flag == 3:
                traffic_light_1.set_color(color=QLabsTrafficLight.COLOR_YELLOW)
                traffic_light_3.set_color(color=QLabsTrafficLight.COLOR_YELLOW)
                traffic_light_2.set_color(color=QLabsTrafficLight.COLOR_RED)
                traffic_light_4.set_color(color=QLabsTrafficLight.COLOR_RED)

            intersection_flag = (intersection_flag + 1) % 4
            time.sleep(interval_s)
        except Exception as exc:
            print(f"Traffic light sequence error: {exc}")
            time.sleep(1)


def pedestrian_patrol(person, start_location, end_location, speed):
    """
    Controls a person to walk back and forth.
    Calculates wait time manually to avoid timeouts at low speeds.
    """
    # 1. Calculate the total distance between points
    distance = math.sqrt(
        (end_location[0] - start_location[0]) ** 2
        + (end_location[1] - start_location[1]) ** 2
        + (end_location[2] - start_location[2]) ** 2
    )

    # 2. Calculate time required (Time = Distance / Speed)
    # Add a 20% buffer to be safe
    travel_time = (distance / speed) * 1.2

    # print(f"Pedestrian Travel Time: {travel_time:.2f} seconds")

    while True:
        try:
            # --- Move to End ---
            # Set waitForConfirmation=False so the library doesn't time out
            person.move_to(location=end_location, speed=speed, waitForConfirmation=True)

            # Wait manually for the calculated travel time
            time.sleep(travel_time)

            # Optional: Wait a few seconds at the destination
            time.sleep(2)

            # --- Move to Start ---
            person.move_to(
                location=start_location, speed=speed, waitForConfirmation=True
            )

            # Wait manually for the calculated travel time
            time.sleep(travel_time)

            # Optional: Wait a few seconds at the start
            time.sleep(2)
        except Exception as exc:
            print(f"Pedestrian patrol error: {exc}")
            time.sleep(1)


# --- Main Script Execution ---
if __name__ == "__main__":
    qlabs = None
    try:
        print("Connecting to QLabs...")
        qlabs = QuanserInteractiveLabs()
        if not qlabs.open("localhost"):
            print("FATAL: Unable to connect to QLabs. Is the simulation running?")
            sys.exit(1)
        print("Connection successful.")

        # --- FIX: REMOVED THE WORLD RESET COMMAND ---
        # qlabs.destroy_all_spawned_actors()  <-- This line was deleting your cars!

        hSystem = QLabsSystem(qlabs)
        hSystem.set_title_string(
            "Environment Logic - Node Map", waitForConfirmation=True
        )
        hEnvironmentOutdoors2 = QLabsEnvironmentOutdoors(qlabs)

        # == 1. SETUP PHASE: Build the Node Following Map ==
        # This will spawn walls/floors around the cars that initCars.py already placed
        setup_node_following_map(qlabs)

        # Spawn Traffic Lights (Matching node_following_only.py coordinates)
        tl1 = QLabsTrafficLight(qlabs)
        tl2 = QLabsTrafficLight(qlabs)
        tl3 = QLabsTrafficLight(qlabs)
        tl4 = QLabsTrafficLight(qlabs)

        # Use id_degrees to ensure they get the specific IDs (1, 2, 3, 4) expected by your logic
        tl1.spawn_id_degrees(
            1,
            [6.0, 15.5, 0.06],
            [0, 0, 0],
            [1, 1, 1],
            configuration=0,
            waitForConfirmation=True,
        )
        tl2.spawn_id_degrees(
            2,
            [-6.0, 12.8, 0.06],
            [0, 0, 90],
            [1, 1, 1],
            configuration=0,
            waitForConfirmation=True,
        )
        tl3.spawn_id_degrees(
            3,
            [-3.7, 3.0, 0.06],
            [0, 0, 180],
            [1, 1, 1],
            configuration=0,
            waitForConfirmation=True,
        )
        tl4.spawn_id_degrees(
            4,
            [7.5, 4.8, 0.06],
            [0, 0, -90],
            [1, 1, 1],
            configuration=0,
            waitForConfirmation=True,
        )

        
        PED_1_START = [-5.52, 13.39, 0.05]
        PED_1_END = [-5.52, 5.27, 0.05]
        PED_1_START_ALT = [-45.69, 15.82, 0]
        PED_1_END_ALT = [42.29, 15.48, 0]
        PED_2_START = [-3.02, 3.46, 0.06]
        PED_2_END = [5.79, 3.46, 0.06]
        PED_3_START = [-3.97, 15.81, 0.06]
        PED_3_END = [5.71, 15.81, 0.06]
        PED_4_START = [7.57, 14.44, 0.06]
        PED_4_END = [7.57, 4.54, 0.06]
        PED_5_START = [14.46, 14.11, 0.06]
        PED_5_END = [14.46, 4.72, 0.06]
        PED_6_START = [-22.15, 2.16, 0.06]
        PED_6_END = [-13.87, 2.16, 0.06]
        # 2. Spawn the person
        person1 = QLabsPerson(qlabs)
        person1.spawn_id(
            actorNumber=100,  # Unique ID (avoid conflict with cars/lights)
            location=PED_1_START,
            rotation=[0, 0, 0],  # Facing +X
            scale=[1, 1, 1],
            configuration=9,  # 0=Casual Male, 1=Casual Female, etc.
            waitForConfirmation=True,
        )
        person1.enable_collsion(enable=True, waitForConfirmation=True)
        person1.add_collision_filter(161, waitForConfirmation=True)  # Filter for cars
        person2 = QLabsPerson(qlabs)
        person2.spawn_id(
            actorNumber=101,  # Unique ID (avoid conflict with cars/lights)
            location=PED_2_START,
            rotation=[0, 0, 0],  # Facing +X
            scale=[1, 1, 1],
            configuration=9,  # 0=Casual Male, 1=Casual Female, etc.
            waitForConfirmation=True,
        )
        person2.enable_collsion(enable=True, waitForConfirmation=True)
        person2.add_collision_filter(161, waitForConfirmation=True)  # Filter for cars
        person3 = QLabsPerson(qlabs)
        person3.spawn_id(
            actorNumber=102,  # Unique ID (avoid conflict with cars/lights)
            location=PED_3_START,
            rotation=[0, 0, 0],  # Facing +X
            scale=[1, 1, 1],
            configuration=9,  # 0=Casual Male, 1=Casual Female, etc.
            waitForConfirmation=True,
        )
        person3.enable_collsion(enable=True, waitForConfirmation=True)
        person3.add_collision_filter(161, waitForConfirmation=True)  # Filter for cars
        person4 = QLabsPerson(qlabs)
        person4.spawn_id(
            actorNumber=103,  # Unique ID (avoid conflict with cars/lights)
            location=PED_4_START,
            rotation=[0, 0, 0],  # Facing +X
            scale=[1, 1, 1],
            configuration=9,  # 0=Casual Male, 1=Casual Female, etc.
            waitForConfirmation=True,
        )
        person4.enable_collsion(enable=True, waitForConfirmation=True)
        person4.add_collision_filter(161, waitForConfirmation=True)  # Filter for cars
        person5 = QLabsPerson(qlabs)
        person5.spawn_id(
            actorNumber=104,  # Unique ID (avoid conflict with cars/lights)
            location=PED_5_START,
            rotation=[0, 0, 0],  # Facing +X
            scale=[1, 1, 1],
            configuration=9,  # 0=Casual Male, 1=Casual Female, etc.
            waitForConfirmation=True,
        )
        person5.enable_collsion(enable=True, waitForConfirmation=True)
        person5.add_collision_filter(161, waitForConfirmation=True)  # Filter for cars
        person6 = QLabsPerson(qlabs)
        person6.spawn_id(
            actorNumber=105,  # Unique ID (avoid conflict with cars/lights)
            location=PED_6_START,
            rotation=[0, 0, 0],  # Facing +X
            scale=[1, 1, 1],
            configuration=9,  # 0=Casual Male, 1=Casual Female, etc.
            waitForConfirmation=True,
        )
        person6.enable_collsion(enable=True, waitForConfirmation=True)
        person6.add_collision_filter(161, waitForConfirmation=True)  # Filter for cars
        ped_1_thread = threading.Thread(
            target=pedestrian_patrol,
            args=(person1, PED_1_START, PED_1_END, 1.0),  # 1.0 m/s walking speed
            daemon=True,
        )
        ped_1_thread.start()
        ped_2_thread = threading.Thread(
            target=pedestrian_patrol,
            args=(person2, PED_2_START, PED_2_END, 1.0),  # 1.0 m/s walking speed
            daemon=True,
        )
        ped_2_thread.start()
        ped_3_thread = threading.Thread(
            target=pedestrian_patrol,
            args=(person3, PED_3_START, PED_3_END, 1.0),  # 1.0 m/s walking speed
            daemon=True,
        )
        ped_3_thread.start()
        ped_4_thread = threading.Thread(
            target=pedestrian_patrol,
            args=(person4, PED_4_START, PED_4_END, 1.0),  # 1.0 m/s walking speed
            daemon=True,
        )
        ped_4_thread.start()
        ped_5_thread = threading.Thread(
            target=pedestrian_patrol,
            args=(person5, PED_5_START, PED_5_END, 1.0),  # 1.0 m/s walking speed
            daemon=True,
        )
        ped_5_thread.start()
        ped_6_thread = threading.Thread(
            target=pedestrian_patrol,
            args=(person6, PED_6_START, PED_6_END, 1.0),  # 1.0 m/s walking speed
            daemon=True,
        )
        ped_6_thread.start()

        Thread(
            target=traffic_light_logic,
            args=(tl1, tl2, tl3, tl4),
            daemon=True,
        ).start()

        print("Traffic light sequence running. Press Ctrl+C in the console to quit.")
        while True:
            time.sleep(1)

    except KeyboardInterrupt:
        print("\nScript terminated by user.")
    except Exception as e:
        print(f"\nAn unexpected error occurred: {e}")
    finally:
        print("Shutting down...")
        if qlabs and qlabs.is_open():
            qlabs.close()
        sys.exit(0)
