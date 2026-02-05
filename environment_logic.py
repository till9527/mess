import sys
import time
import math
from threading import Thread, Event  # Import Event
import tkinter as tk
from functools import partial

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


def set_light_color(light_handle, color_const, color_name):
    """Callback function to set a specific light to a specific color."""
    print(f"Setting light {light_handle.actorNumber} to {color_name}")
    light_handle.set_color(color_const)


# --- Logic Functions for Threading ---


def traffic_light_sequence(
    traffic_light, stop_event, red_time=15, green_time=30, yellow_time=1, delay=0
):
    """
    Controls the R-Y-G sequence for a single traffic light in a continuous loop.
    Checks a stop_event to allow for graceful termination.
    """
    try:
        # time.sleep(delay)
        # Use wait() instead of sleep() so it can be interrupted
        if stop_event.wait(delay):
            print(
                f"Light {traffic_light.actorNumber} sequence stopped during initial delay."
            )
            return  # Stop requested before we even started

        while not stop_event.is_set():
            # Check flag before each action
            if stop_event.is_set():
                break
            traffic_light.set_color(QLabsTrafficLight.COLOR_GREEN)
            # wait() returns True if event was set, False if it timed out
            if stop_event.wait(green_time):
                break  # Stop requested

            if stop_event.is_set():
                break
            traffic_light.set_color(QLabsTrafficLight.COLOR_YELLOW)
            if stop_event.wait(yellow_time):
                break  # Stop requested

            if stop_event.is_set():
                break
            traffic_light.set_color(QLabsTrafficLight.COLOR_RED)
            if stop_event.wait(red_time):
                break  # Stop requested

        print(f"Traffic light {traffic_light.actorNumber} sequence stopped.")

    except Exception as e:
        print(f"Error in traffic_light_sequence for {traffic_light.actorNumber}: {e}")
        # This can happen if the QLabs connection is closed while the thread is waiting
        pass


def pedestrian_patrol(person, start_location, end_location, speed):
    """Controls a person to walk back and forth between two points."""
    while True:
        try:
            person.move_to(location=end_location, speed=speed, waitForConfirmation=True)
            time.sleep(10)
            person.move_to(
                location=start_location, speed=speed, waitForConfirmation=True
            )
            time.sleep(10)
        except Exception as e:
            print(f"Pedestrian patrol interrupted (likely simulation shutdown): {e}")
            break  # Exit loop on error


# --- UI Control Class ---
# Add near your other logic functions


def weather_sequence(environment_handle):
    """
    Cycles through different weather presets at a given interval using system time.
    """
    WEATHER_PRESETS = [
        ("Clear", QLabsEnvironmentOutdoors.CLEAR_SKIES),
        ("Cloudy", QLabsEnvironmentOutdoors.CLOUDY),
        ("Rain", QLabsEnvironmentOutdoors.RAIN),
        ("Snow", QLabsEnvironmentOutdoors.SNOW),
    ]

    preset_index = 0
    num_presets = len(WEATHER_PRESETS)

    try:
        while True:
            # Apply the weather BEFORE waiting
            weather_name, weather_const = WEATHER_PRESETS[preset_index]
            environment_handle.set_weather_preset(weather_const)
            print(f"Weather set to: {weather_name}")

            # Increment index for the NEXT cycle
            preset_index = (preset_index + 1) % num_presets
            time.sleep(15)

            # Use stop_event.wait() to sleep, allowing for immediate interruption on shutdown

        print("Weather sequence stopped.")

    except Exception as e:
        print(f"Error in weather_sequence: {e}")


class TrafficControlApp:
    def __init__(self, root, light_handles):
        self.root = root
        self.light_4_handle = light_handles[0]
        self.light_3_handle = light_handles[1]

        self.light_threads = []
        self.stop_events = []

        # State variable for the toggle
        self.auto_mode_var = tk.BooleanVar(value=True)

        self.setup_ui()
        self.on_toggle()  # Call once to set initial state (auto mode)

    def setup_ui(self):
        # --- Frame for Toggle ---
        toggle_frame = tk.Frame(self.root)
        toggle_frame.pack(fill=tk.X, padx=10, pady=5)

        self.toggle_button = tk.Checkbutton(
            toggle_frame,
            text="Automatic Mode",
            variable=self.auto_mode_var,
            command=self.on_toggle,
            font=("Helvetica", 10, "bold"),
        )
        self.toggle_button.pack()

        # --- Frame for Light 4 ---
        frame4 = tk.Frame(self.root, relief=tk.RIDGE, borderwidth=2)
        frame4.pack(fill=tk.X, padx=10, pady=5)
        tk.Label(frame4, text="Light ID 4 (Positive X)").pack()

        self.btn_4_g = tk.Button(
            frame4,
            text="Green",
            bg="#70e070",
            command=partial(
                set_light_color,
                self.light_4_handle,
                QLabsTrafficLight.COLOR_GREEN,
                "GREEN",
            ),
        )
        self.btn_4_g.pack(side=tk.LEFT, expand=True, fill=tk.X)

        self.btn_4_y = tk.Button(
            frame4,
            text="Yellow",
            bg="#f0e070",
            command=partial(
                set_light_color,
                self.light_4_handle,
                QLabsTrafficLight.COLOR_YELLOW,
                "YELLOW",
            ),
        )
        self.btn_4_y.pack(side=tk.LEFT, expand=True, fill=tk.X)

        self.btn_4_r = tk.Button(
            frame4,
            text="Red",
            bg="#f07070",
            command=partial(
                set_light_color, self.light_4_handle, QLabsTrafficLight.COLOR_RED, "RED"
            ),
        )
        self.btn_4_r.pack(side=tk.LEFT, expand=True, fill=tk.X)

        # --- Frame for Light 3 ---
        frame3 = tk.Frame(self.root, relief=tk.RIDGE, borderwidth=2)
        frame3.pack(fill=tk.X, padx=10, pady=5)
        tk.Label(frame3, text="Light ID 3 (Negative X)").pack()

        self.btn_3_g = tk.Button(
            frame3,
            text="Green",
            bg="#70e070",
            command=partial(
                set_light_color,
                self.light_3_handle,
                QLabsTrafficLight.COLOR_GREEN,
                "GREEN",
            ),
        )
        self.btn_3_g.pack(side=tk.LEFT, expand=True, fill=tk.X)

        self.btn_3_y = tk.Button(
            frame3,
            text="Yellow",
            bg="#f0e070",
            command=partial(
                set_light_color,
                self.light_3_handle,
                QLabsTrafficLight.COLOR_YELLOW,
                "YELLOW",
            ),
        )
        self.btn_3_y.pack(side=tk.LEFT, expand=True, fill=tk.X)

        self.btn_3_r = tk.Button(
            frame3,
            text="Red",
            bg="#f07070",
            command=partial(
                set_light_color, self.light_3_handle, QLabsTrafficLight.COLOR_RED, "RED"
            ),
        )
        self.btn_3_r.pack(side=tk.LEFT, expand=True, fill=tk.X)

        # Store buttons for easy access
        self.manual_buttons = [
            self.btn_4_g,
            self.btn_4_y,
            self.btn_4_r,
            self.btn_3_g,
            self.btn_3_y,
            self.btn_3_r,
        ]

    def set_manual_buttons_state(self, state):
        """Helper function to enable/disable all manual buttons."""
        for btn in self.manual_buttons:
            btn.config(state=state)

    def on_toggle(self):
        """Called when the toggle button is clicked."""
        if self.auto_mode_var.get():
            # --- SWITCHING TO AUTOMATIC ---
            print("Switching to AUTOMATIC mode.")
            self.set_manual_buttons_state(tk.DISABLED)
            self.start_auto_sequences()
        else:
            # --- SWITCHING TO MANUAL ---
            print("Switching to MANUAL mode.")
            self.stop_auto_sequences()
            self.set_manual_buttons_state(tk.NORMAL)

    def start_auto_sequences(self):
        """Stops any existing sequences and starts new ones."""
        self.stop_auto_sequences()  # Ensure old ones are stopped

        print("Starting automatic light sequences...")

        # Create event and thread for Light 4
        stop_event_4 = Event()
        t4 = Thread(
            target=traffic_light_sequence,
            args=(self.light_4_handle, stop_event_4, 15, 30, 1, 0),  # 0s delay
            daemon=True,  # Make daemon so it exits if main thread dies
        )

        # Create event and thread for Light 3
        stop_event_3 = Event()
        t3 = Thread(
            target=traffic_light_sequence,
            args=(
                self.light_3_handle,
                stop_event_3,
                15,
                30,
                1,
                16,
            ),  # 16s delay (Yellow + Red)
            daemon=True,
        )

        self.stop_events = [stop_event_4, stop_event_3]
        self.light_threads = [t4, t3]

        t4.start()
        t3.start()

    def stop_auto_sequences(self):
        """Signals all running traffic light threads to stop."""
        if not self.light_threads:
            return  # Nothing to stop

        print("Stopping automatic light sequences...")

        for event in self.stop_events:
            event.set()  # Signal threads to stop

        for thread in self.light_threads:
            thread.join(timeout=0.5)  # Wait briefly for them to exit

        self.light_threads = []
        self.stop_events = []


# --- Main Script Execution ---
if __name__ == "__main__":
    qlabs = None
    app = None
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

        # Spawn Pedestrian
        NEW_CW_START = [14.5, 6.0, 0.2]
        NEW_CW_END = [14.5, 13.0, 0.2]

        person1 = QLabsPerson(qlabs)
        person1.spawn_id(
            actorNumber=0,
            location=NEW_CW_START,
            rotation=[0, 0, math.pi / 2],
            scale=[1, 1, 1],
            configuration=9,
            waitForConfirmation=True,
        )

        print("All actors have been spawned.")

        # == 2. LOGIC PHASE ==
        print("Starting background logic...")

        # Start pedestrian logic
        Thread(
            target=pedestrian_patrol,
            args=(person1, NEW_CW_START, NEW_CW_END, 1.2),
            daemon=True,
        ).start()

        Thread(
            target=weather_sequence,
            args=(hEnvironmentOutdoors2,),
            daemon=True,
        ).start()

        # == 3. UI CONTROL PHASE ==
        print("Launching Traffic Light Control UI...")
        root = tk.Tk()
        root.title("Traffic Light Control")

        app = TrafficControlApp(root, [tl4, tl3])

        root.protocol("WM_DELETE_WINDOW", lambda: print("Use Ctrl+C to stop."))
        print("UI is running. Press Ctrl+C in the console to quit.")
        root.mainloop()

    except KeyboardInterrupt:
        print("\nScript terminated by user.")
    except Exception as e:
        print(f"\nAn unexpected error occurred: {e}")
    finally:
        print("Shutting down...")
        if app:
            app.stop_auto_sequences()
        if qlabs and qlabs.is_open():
            qlabs.close()
        sys.exit(0)
