#!/usr/bin/env python3

from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor
from pybricks.parameters import Port, Direction, Button, Color
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch

# ==============================================================================
# 1. HARDWARE SETUP & CONSTANTS
# ==============================================================================

# Hub initialization
hub = PrimeHub()

# Drive Motors (Ports A and E)
left_motor = Motor(Port.A)
right_motor = Motor(Port.E, Direction.COUNTERCLOCKWISE)

# DriveBase configuration
# Wheel circumference = 19.6 cm -> Diameter = 196mm / pi ≈ 62.4 mm
# Axle track = Distance between wheel centers in mm (Adjust to your robot)
robot = DriveBase(left_motor, right_motor, wheel_diameter=62.4, axle_track=150)

# Enable built-in gyro control for straight driving and turns
robot.use_gyro(True)

# Attachment Motors (Ports B and C)
attachment_b = Motor(Port.B)  # Main Arm / Lift Motor
attachment_c = Motor(Port.C)  # Auxiliary Arm / Hammer / Flag Motor

# ==============================================================================
# 2. UTILITY FUNCTIONS
# ==============================================================================

def reset_gyro():
    """Resets hub gyro sensor heading to zero."""
    hub.imu.reset_heading(0)
    wait(100)

def set_drive_speed(speed=500, accel=400, turn_speed=200, turn_accel=300):
    """Sets drive speeds in mm/s and turn rates in deg/s."""
    robot.settings(straight_speed=speed, straight_acceleration=accel, 
                   turn_rate=turn_speed, turn_acceleration=turn_accel)

# ==============================================================================
# 3. MISSION RUN FUNCTIONS
# ==============================================================================

def run_1():
    # Turn right to align with forum
    robot.turn(4)

    # Go major distance backwards (fast) - 75cm (750mm)
    set_drive_speed(speed=800)
    robot.straight(-750)

    # Go all the way backwards (slower) to align with back walls - 15cm (150mm)
    set_drive_speed(speed=200)
    robot.straight(-150)

    # Go forward 2.5cm (25mm)
    set_drive_speed(speed=300)
    robot.straight(25)

    # (In Parallel) Lower arm for mineshaft explorer
    attachment_c.run_angle(speed=300, rotation_angle=375, wait=False)

    # Turn left to face precious-artifact (-90° heading)
    robot.turn(-94)

    # Go forward to make contact - 5cm (50mm)
    set_drive_speed(speed=125)
    robot.straight(50)

    # Go forward to touch precious-artifact - 6.25cm (62.5mm)
    set_drive_speed(speed=75)
    robot.straight(62.5)

    # Lift arm slightly to lift precious-artifact
    attachment_b.run_angle(speed=400, rotation_angle=195, wait=True)

    # Lift arm to operate "Mineshaft Explorer"
    attachment_c.run_angle(speed=150, rotation_angle=-360, wait=True)

    # Go backward slightly to snatch artifact - 18.5cm (185mm)
    set_drive_speed(speed=350)
    robot.straight(-185)

    # Lift arm to slide precious-artifact (in background)
    attachment_b.run_angle(speed=600, rotation_angle=145, wait=False)

    # Turn right to align with forum
    robot.turn(48)

    # (In Parallel) Lower arm to drop off precious-artifact
    attachment_b.run_angle(speed=1100, rotation_angle=-600, wait=False)

    # Go forward to forum - 22cm (220mm)
    set_drive_speed(speed=800)
    robot.straight(220)

    # Lower arm in opposite direction (in background)
    attachment_c.run_angle(speed=400, rotation_angle=-360, wait=False)
    wait(100)

    # Go backwards away from forum - 13cm (130mm)
    set_drive_speed(speed=600)
    robot.straight(-130)

    # Go backwards - 12.5cm (125mm)
    set_drive_speed(speed=250)
    robot.straight(-125)

    # Lift arm to pick up top soil in two stages
    attachment_c.run_angle(speed=100, rotation_angle=90, wait=True)
    attachment_c.run_angle(speed=100, rotation_angle=125, wait=True)

    # Move slightly forward - 3cm (30mm)
    set_drive_speed(speed=400)
    robot.straight(30)

    # Turn right to face base
    robot.turn(50)

    # Go forward to base - 70cm (700mm)
    set_drive_speed(speed=800)
    robot.straight(700)


def run_2():
    # Approach Map Reveal part 1 - 50cm (500mm)
    set_drive_speed(speed=700)
    robot.straight(500)

    # Approach Map Reveal slowly - 22cm (220mm)
    set_drive_speed(speed=500)
    robot.straight(220)

    # Move backward to flick surface brushing - 16cm (160mm)
    robot.straight(-160)
    wait(250)

    # Drop topsoil into forum
    attachment_c.run_angle(speed=200, rotation_angle=260, wait=True)

    # Raise Surface Brushing Brush
    attachment_b.run_angle(speed=600, rotation_angle=-1000, wait=False)

    # Raise topsoil arm
    attachment_c.run_angle(speed=500, rotation_angle=-260, wait=True)

    # Go forward to approach map reveal - 18cm (180mm)
    set_drive_speed(speed=400)
    robot.straight(180)

    # Turn left -40 degrees
    robot.turn(-40)

    # Move Map Reveal piece - 13cm (130mm)
    robot.straight(130)

    # Lower arm to push back top soil piece
    attachment_c.run_angle(speed=300, rotation_angle=385, wait=True)

    # Complete moving Map Reveal piece - 4cm (40mm)
    set_drive_speed(speed=100)
    robot.straight(40)

    # Lift arm that pushed back top soil
    attachment_c.run_angle(speed=650, rotation_angle=-350, wait=True)

    # Lower surface brush in parallel
    attachment_b.run_angle(speed=700, rotation_angle=200, wait=False)

    # Move backward away from Map Reveal - 21cm (210mm)
    set_drive_speed(speed=600)
    robot.straight(-210)

    # Turn left toward base (-110° relative turn to reach -150°)
    robot.turn(-110)

    # Go backward to drop surface brush - 8cm (80mm)
    robot.straight(-80)

    # Drop surface brush in forum
    attachment_b.run_angle(speed=1100, rotation_angle=650, wait=True)

    # Go forward to base - 72cm (720mm)
    set_drive_speed(speed=800)
    robot.straight(720)


def run_3():
    # Approach salvage operation - 36.5cm (365mm)
    set_drive_speed(speed=600)
    robot.straight(365)

    # Stepwise speed ramp-up to approach salvage
    for dist, speed in [(100, 200), (100, 400), (100, 600), (100, 800), (100, 800)]:
        set_drive_speed(speed=speed)
        robot.straight(dist)

    # Move arm down to drop flag
    attachment_c.run_angle(speed=400, rotation_angle=300, wait=True)

    # Go back to base slower - 14cm (140mm)
    set_drive_speed(speed=150)
    robot.straight(-140)

    # Move flag arm up to release
    attachment_c.run_angle(speed=400, rotation_angle=-300, wait=True)

    # Go back to base fast - 35cm (350mm)
    set_drive_speed(speed=800)
    robot.straight(-350)


def run_4():
    # Bring arm down to start engaging statue rebuild
    attachment_b.run_angle(speed=1100, rotation_angle=-2300, wait=False)

    # Turn left to avoid salvage operation
    robot.turn(-20)

    # Approach statue rebuild - 15cm (150mm)
    set_drive_speed(speed=500)
    robot.straight(-150)

    # Turn right to align with statue rebuild (+153° relative)
    robot.turn(153)

    # Go forward to statue rebuild - 27.5cm (275mm)
    robot.straight(275)

    # Bring arm down to engage statue rebuild
    attachment_b.run_angle(speed=1100, rotation_angle=-550, wait=True)

    # Turn right to get lever under statue rebuild (+9° relative)
    set_drive_speed(turn_speed=75)
    robot.turn(9)
    wait(100)

    # Bring arm up to lift statue
    attachment_b.run_angle(speed=1100, rotation_angle=1200, wait=True)

    # Move backward away from statue rebuild - 9cm (90mm)
    set_drive_speed(straight_speed=400, turn_speed=200)
    robot.straight(-90)

    # Bring arm up & turn right toward scale (-141° relative)
    attachment_b.run_angle(speed=1100, rotation_angle=1700, wait=False)
    robot.turn(-141)

    # Align with tip the scale - 37cm (370mm)
    set_drive_speed(speed=600)
    robot.straight(-370)

    # Turn to 5° and drive backward 39cm (390mm)
    robot.turn(5)
    robot.straight(-390)

    # Turn left to align with scale (-91° relative)
    robot.turn(-91)

    # Latch with scale - 30cm (300mm)
    set_drive_speed(speed=200)
    robot.straight(-300)

    # Go forward to pull pan - 12cm (120mm)
    robot.straight(120)

    # Align with angler artifact (-13° relative)
    robot.turn(-13)

    # Lift angler artifact
    attachment_c.run_angle(speed=250, rotation_angle=-500, wait=True)

    # Turn to un-latch gear (+13° relative)
    robot.turn(13)

    # Go forward away from artifact - 1cm (10mm)
    robot.straight(-10)

    # Turn right to align with market wares (+68° relative)
    robot.turn(68)

    # Drive backward to market wares - 30cm (300mm)
    set_drive_speed(speed=600)
    robot.straight(-300)

    # Complete market wares - 15cm (150mm)
    robot.straight(150)

    # Turn right to escape (+42° relative)
    robot.turn(42)

    # Go backwards to base - 55cm (550mm)
    set_drive_speed(speed=800)
    robot.straight(-550)


def run_5():
    # Approach silo - 41.5cm (415mm)
    set_drive_speed(speed=500)
    robot.straight(415)

    # Hit silo lever 4 times
    for _ in range(4):
        attachment_c.run_angle(speed=940, rotation_angle=230, wait=True)
        wait(60)
        attachment_c.run_angle(speed=700, rotation_angle=-230, wait=True)

    # Bring heavy lifting arm down
    attachment_b.run_angle(speed=1100, rotation_angle=-1650, wait=False)

    # Approach 'who lived here' - 30.5cm (305mm)
    set_drive_speed(speed=350)
    robot.straight(305)

    # Turn left (-30° relative)
    robot.turn(-30)

    # Go backward to align - 11cm (110mm)
    robot.straight(-110)

    # Turn right to align with forge (+75° relative)
    robot.turn(75)

    # Bring heavy lifting arm down (2)
    attachment_b.run_angle(speed=1100, rotation_angle=-550, wait=True)

    # Engage heavy lifting - 9cm (90mm)
    set_drive_speed(speed=180)
    robot.straight(90)

    # Bring heavy lifting arm up
    attachment_b.run_angle(speed=1000, rotation_angle=900, wait=True)
    attachment_b.run_angle(speed=1000, rotation_angle=1300, wait=False)

    # Go backwards from forge - 30cm (300mm)
    set_drive_speed(speed=600)
    robot.straight(-300)

    # Turn left toward base (-58° relative)
    robot.turn(-58)

    # Return to base - 75cm (750mm)
    set_drive_speed(speed=800)
    robot.straight(-750)


def run_6():
    # Turn left out of base (-25° relative)
    robot.turn(-25)

    # Align with opposing mineshaft - 68cm (680mm)
    set_drive_speed(speed=650)
    robot.straight(680)

    # Turn left (-10° relative)
    robot.turn(-10)

    # Drive forward 5cm (50mm)
    robot.straight(50)

    # Turn left (-53° relative)
    robot.turn(-53)

    # Drive forward 28cm (280mm)
    robot.straight(280)

    # Turn right to align with flag dropoff (+88° relative)
    robot.turn(88)

    # Drop flag - 15cm (150mm)
    set_drive_speed(speed=250)
    robot.straight(150)

    # Lift opposing team mineshaft
    attachment_c.run_angle(speed=1100, rotation_angle=1000, wait=True)

    # Go backward - 7cm (70mm)
    robot.straight(-70)

    # Turn left to align with what's on sale (-43° relative)
    robot.turn(-43)

    # Push roof for what's on sale - 20cm (200mm) + 11.5cm (115mm)
    set_drive_speed(speed=400)
    robot.straight(-200)
    robot.straight(-115)

    # Move forward - 7cm (70mm)
    robot.straight(70)

    # Turn left (-45° relative)
    robot.turn(-45)

    # Align with forum - 32cm (320mm)
    set_drive_speed(speed=500)
    robot.straight(320)

    # Turn left (-15° relative) and drive 16cm (160mm)
    robot.turn(-15)
    robot.straight(160)

    # Turn left (-45° relative) and drive 2cm (20mm)
    robot.turn(-45)
    robot.straight(20)

    # Drop opposing mineshaft in forum
    attachment_c.run_angle(speed=1100, rotation_angle=-1400, wait=False)

    # Turn left (-15° relative)
    robot.turn(-15)

    # Drop scale pan and heavy lifting onto forum
    attachment_b.run_angle(speed=1100, rotation_angle=-1300, wait=True)

    # Turn right (+18° relative)
    robot.turn(18)

    # Go backwards - 3cm (30mm)
    robot.straight(-30)

    # Turn right to align (+62° relative)
    robot.turn(62)

    # Drive forward to drop flag - 15cm (150mm)
    set_drive_speed(speed=600)
    robot.straight(150)

# ==============================================================================
# 4. FLL MAIN MENU SELECTOR SYSTEM
# ==============================================================================

def execute_sequence(run_list):
    """Executes a list of runs sequentially with timer tracking."""
    timer = StopWatch()
    total_time = 0

    for run_num in run_list:
        run_map = {1: run_1, 2: run_2, 3: run_3, 4: run_4, 5: run_5, 6: run_6}

        if run_num in run_map:
            # Display run number on Hub screen
            hub.display.char(str(run_num))
            hub.light.on(Color.RED)

            # Wait for Left button press to start the run
            while Button.LEFT not in hub.buttons.pressed():
                wait(20)

            # Pause briefly to prevent immediate re-triggering
            wait(300)
            hub.light.on(Color.MAGENTA)

            # Reset gyro heading before starting the mission run
            reset_gyro()

            # Execute run and track time
            timer.reset()
            run_map[run_num]()
            run_time = timer.time() / 1000.0

            total_time += run_time
            print("Run {} Completed in {:.2f}s".format(run_num, run_time))
            hub.light.on(Color.GREEN)

    print("==========================================")
    print("ALL RUNS COMPLETE | Total Time: {:.2f}s".format(total_time))
    print("==========================================")


# ==============================================================================
# 5. ENTRY POINT
# ==============================================================================

# Run all missions (1 to 6) in sequence with button prompts
execute_sequence([1, 2, 3, 4, 5, 6])
