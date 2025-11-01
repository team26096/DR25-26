#!/usr/bin/env python3

import hub
import sys
import time

import color, motor, motor_pair, runloop
from hub import light_matrix, button, motion_sensor, light, port


# CONSTANTS
#----------------------------------------

WHEEL_CIRCUMFERENCE = 19.6

# END CONSTANTS
#----------------------------------------


# UTILITY FUNCTIONS
#----------------------------------------

# initialize motor and reset yaw
def do_init():
    # reset yaw to 0
    motion_sensor.set_yaw_face(motion_sensor.TOP)
    motion_sensor.reset_yaw(0)
    i = 0
    while (hub.motion_sensor.stable() == False):
        i = i + 1
        # Use time.sleep_ms instead of time.sleep_ms
        # to ensure it is synchronized
        time.sleep_ms(10)
        hub.light_matrix.write(str(i))
        if i >= 100:
            break


# Return true if LEFT button is pressed
def is_left_button_pressed():
    return button.pressed(button.LEFT) > 0


# Return true if RIGHT button is pressed
def is_right_button_pressed():
    return button.pressed(button.RIGHT) > 0


def follow_for_distance(initial_position=0,
                        distance_to_cover=0):
    current_position = abs(motor.relative_position(port.A))
    distance_covered = current_position - initial_position
    if distance_covered < 0 : distance_covered = distance_covered * -1
    if (distance_covered >= abs(distance_to_cover)):
        return False
    else:
        return True


def get_yaw_value():
    return motion_sensor.tilt_angles()[0] * -0.1


def degrees_for_distance(distance_cm):
    # Add multiplier for gear ratio if needed
    return int((distance_cm/WHEEL_CIRCUMFERENCE) * 360)


def wait_for_yaw_abs(angle=0):
    abs_angle = abs(angle)
    abs_current_yaw = abs(get_yaw_value())
    if angle == 0:
        if get_yaw_value() > 0:
            while get_yaw_value() >= angle: time.sleep_ms(10)
        elif get_yaw_value() < 0:
            while get_yaw_value() <= angle: time.sleep_ms(10)
    elif abs_current_yaw > abs_angle:
        while abs(get_yaw_value()) >= abs_angle: time.sleep_ms(10)
    elif abs_current_yaw < abs_angle:
        while abs(get_yaw_value()) <= abs_angle: time.sleep_ms(10)


async def follow_gyro_angle(kp,
                            ki,
                            kd,
                            speed,
                            target_angle,
                            sleep_time,
                            follow_for, **kwargs):
    # get initial reading from left motor
    integral = 0.0
    last_error = 0.0
    derivative = 0.0
    while (follow_for(**kwargs)):
        current_angle = get_yaw_value()
        error = current_angle - target_angle
        integral = integral + error
        derivative = error - last_error
        last_error = error
        # compute steering correction
        steering_value = (error * kp) + (integral * ki) + (derivative * kd)

        if sleep_time:
            time.sleep_ms(sleep_time)
        # kp value should be +ve for forward movement (positive speed value), and -ve for backward movement (negative speed value)
        motor_pair.move(motor_pair.PAIR_1, int(steering_value), velocity=speed)

    # stop when follow_for condition is met
    motor_pair.stop(motor_pair.PAIR_1, stop=motor.HOLD)


async def pivot_gyro_turn_abs(left_speed=0, right_speed=50, angle=90, stop=False):
    motor_pair.move_tank(motor_pair.PAIR_1, left_speed, right_speed)
    wait_for_yaw_abs(angle=angle)
    if stop: motor_pair.stop(motor_pair.PAIR_1, stop=motor.HOLD)


def get_yaw_angle():
    current_yaw = motion_sensor.tilt_angles()[0] * -0.1
    if (current_yaw < 0):
        return (current_yaw + 360)
    return current_yaw

async def turnRight(angle):
    motor_pair.move_tank(motor_pair.PAIR_1, 200, -200)
    while abs(get_yaw_angle()) <= angle: time.sleep_ms(10)
    motor_pair.stop(motor_pair.PAIR_1, stop=motor.HOLD)

async def turnLeft(angle):
    motor_pair.move_tank(motor_pair.PAIR_1, -200, 200)
    while abs(get_yaw_angle()) >= angle: time.sleep_ms(10)
    motor_pair.stop(motor_pair.PAIR_1, stop=motor.HOLD)


def get_time_taken_in_seconds(start_time, end_time):
    return int(time.ticks_diff(end_time, start_time)/1000)

# END UTILITY FUNCTIONS
#----------------------------------------


# RUN FUNCTIONS
#----------------------------------------
async def run_a():
    # go forward partially to get out of base and approach Map Reveal
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1, ki=-0.0002, kd=-0.2, speed=800, target_angle=0, sleep_time=0, follow_for=follow_for_distance,
        initial_position=initial_position, distance_to_cover=(degrees_for_distance(50)))

    # go forward fully slightly slowly to get out of base and approach Map Reveal
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1, ki=-0.0002, kd=-0.2, speed=500, target_angle=0, sleep_time=0, follow_for=follow_for_distance,
        initial_position=initial_position, distance_to_cover=(degrees_for_distance(20)))

    # raise topsoil hooks to get in position
    motor.run_for_degrees(port.C, -80, 800)

    # turn left to get in alignment with Map reveal
    await pivot_gyro_turn_abs(left_speed=-100, right_speed=0, angle=-40, stop=True)

    # go forward to move Map Reveal piece 1 and 2 to the back
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1, ki=-0.0002, kd=-0.2, speed=450, target_angle=-40, sleep_time=0, follow_for=follow_for_distance,
        initial_position=initial_position, distance_to_cover=(degrees_for_distance(20)))

    # raise topsoil hooks to pick up topsoil piece
    await motor.run_for_degrees(port.C, -310, 400)

    # Move backward to move away from Map reveal
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1, ki=0.0002, kd=0.2, speed=-50, target_angle=-40, sleep_time=0, follow_for=follow_for_distance,
    initial_position=initial_position, distance_to_cover=(degrees_for_distance(4)))

    # Turn left to align straight with "Mineshaft Explorer"
    await pivot_gyro_turn_abs(left_speed=-100, right_speed=0, angle=-50, stop=True)

    # Go backward partial distance faster to reach closer to "Mineshaft Explorer"
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1, ki=0.0002, kd=0.2, speed=-700, target_angle=-90, sleep_time=0, follow_for=follow_for_distance,
    initial_position=initial_position, distance_to_cover=(degrees_for_distance(40)))

    # Go backward slowern for final reach (to avoid escaping top soil) to reach closer to "Mineshaft Explorer"
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1, ki=0.0002, kd=0.2, speed=-200, target_angle=-90, sleep_time=0, follow_for=follow_for_distance,
    initial_position=initial_position, distance_to_cover=(degrees_for_distance(10)))

    # Lift arm to operate "Mineshaft Explorer"
    await motor.run_for_degrees(port.C, 400, 250)

    # Lower arm to make sure it does not get stuck in mission while moving forward
    await motor.run_for_degrees(port.C, -375, 900)

    # go forward to move away from Minshaft Explorer
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1, ki=-0.0002, kd=-0.2, speed=450, target_angle=-90, sleep_time=0, follow_for=follow_for_distance,
        initial_position=initial_position, distance_to_cover=(degrees_for_distance(5)))

    # turn left to get ready to move toward Surface Brushing
    await pivot_gyro_turn_abs(left_speed=-100, right_speed=0, angle=-110, stop=True)

    # Lower Surface Brushing Brush to get it into position
    motor.run_for_degrees(port.B, -260, 500)

    # go forward towards Surface Brushing
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1, ki=-0.0002, kd=-0.2, speed=450, target_angle=-110, sleep_time=0, follow_for=follow_for_distance,
        initial_position=initial_position, distance_to_cover=(degrees_for_distance(35)))

    # turn right to Flick surace Brsuhing to complete Part 1
    await pivot_gyro_turn_abs(left_speed=900, right_speed=-900, angle=-60, stop=True)

    # turn left to get in alignment with Surface Brushing again
    await pivot_gyro_turn_abs(left_speed=-900, right_speed=700, angle=-90, stop=True)

    # # go slightly backward to move away from surface brushing, and get ready to turn
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1, ki=0.0002, kd=0.2, speed=-550, target_angle=-129, sleep_time=0, follow_for=follow_for_distance,
       initial_position=initial_position, distance_to_cover=(degrees_for_distance(5)))

    # # turn right to align for picking up brush
    await pivot_gyro_turn_abs(left_speed=200, right_speed=-200, angle=-70, stop=True)

    # # Go backwards for preparing for brush pickup
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1, ki=0.0002, kd=0.2, speed=-550, target_angle=-70, sleep_time=0, follow_for=follow_for_distance,
       initial_position=initial_position, distance_to_cover=(degrees_for_distance(15)))

    # # Raise Surface Brushing Brush to get it in the correct position
    motor.run_for_degrees(port.B, 12, 500)

    # # Go forward for picking up the brush
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1, ki=-0.0002, kd=-0.2, speed=150, target_angle=-65, sleep_time=0, follow_for=follow_for_distance,
       initial_position=initial_position, distance_to_cover=(degrees_for_distance(14)))

    # # Raise Surface Brushing Brush to lift surface brushing brush
    motor.run_for_degrees(port.B, 150, 800)

    # Go backward to move away from surface brushing
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1, ki=0.0002, kd=0.2, speed=-200, target_angle=-90, sleep_time=0, follow_for=follow_for_distance,
    initial_position=initial_position, distance_to_cover=(degrees_for_distance(10)))

    # turn right to get ready to come back to base
    await pivot_gyro_turn_abs(left_speed=200, right_speed=-200, angle=1, stop=True)

    # While going back, lift arm for mineshaft to keep robot in the base
    # TODO

    # Go backward to go to back to base
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1, ki=0.0002, kd=0.2, speed=-500, target_angle=1, sleep_time=0, follow_for=follow_for_distance,
    initial_position=initial_position, distance_to_cover=(degrees_for_distance(65)))


async def run_c():
    # go forward to get out of base and approach salvage operation
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1, ki=-0.0002, kd=-0.2, speed=500, target_angle=1, sleep_time=0, follow_for=follow_for_distance,
        initial_position=initial_position, distance_to_cover=(degrees_for_distance(33)))

    # go forward to approach selvage operation slower
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1, ki=-0.0002, kd=-0.2, speed=200, target_angle=1, sleep_time=0, follow_for=follow_for_distance,
        initial_position=initial_position, distance_to_cover=(degrees_for_distance(13)))

    # move ore arm to complete
    await motor.run_for_degrees(port.B, 350, 500)

    # go back to base slower
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1, ki=0.0002, kd=0.2, speed=-200, target_angle=1, sleep_time=0, follow_for=follow_for_distance,
        initial_position=initial_position, distance_to_cover=(degrees_for_distance(13)))

    # go back to base faster
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1, ki=0.0002, kd=0.2, speed=-500, target_angle=1, sleep_time=0, follow_for=follow_for_distance,
        initial_position=initial_position, distance_to_cover=(degrees_for_distance(37)))


async def run_d():

    # bring arm down to to start engaging with statue rebuild
    motor.run_for_degrees(port.B, -2000, 11000)

    # turn left to avoid salvage operation
    await pivot_gyro_turn_abs(-100, 100, -45, stop=True)

    # go forward to approach tip the scale
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1, ki=-0.0002, kd=-0.2, speed=600, target_angle=-45, sleep_time=0, follow_for=follow_for_distance,
        initial_position=initial_position, distance_to_cover=(degrees_for_distance(30)))

    # bring arm down to to start engaging with statue rebuild
    await motor.run_for_degrees(port.B, -1000, 1100)

    # turn right to align with statue rebuild
    await pivot_gyro_turn_abs(50, -50, -38, stop=True)

    # bring arm up to lift the statue
    await motor.run_for_degrees(port.B, 850, 400)

    await motor_pair.move_for_degrees(motor_pair.PAIR_1, degrees_for_distance(6), 0)

    # go forward to complete statue rebuild
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1, ki=0.0002, kd=0.2, speed=-400, target_angle=-38, sleep_time=0, follow_for=follow_for_distance,
        initial_position=initial_position, distance_to_cover=(degrees_for_distance(9)))

    # turn left to start aligning with tip the scale
    await pivot_gyro_turn_abs(-100, 100, -174, stop=True)

    # bring statue rebuild arm up to original position
    motor.run_for_degrees(port.B, 2600, 800)

    # initialize gyro
    do_init()

    # go forward to approach tip the scale
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1, ki=0.0002, kd=0.2, speed=-600, target_angle=-2, sleep_time=0, follow_for=follow_for_distance,
        initial_position=initial_position, distance_to_cover=(degrees_for_distance(73)))

    # turn left to start aligning with tip the scales
    await pivot_gyro_turn_abs(-100, 100, -95, stop=True)

    # go backward to get align and latch with tip the scale
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1, ki=0.0002, kd=0.2, speed=-200, target_angle=-95, sleep_time=0, follow_for=follow_for_distance,
        initial_position=initial_position, distance_to_cover=(degrees_for_distance(14)))

    # go away from tip the scale and pull scale pan
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1, ki=-0.0002, kd=-0.2, speed=300, target_angle=-92, sleep_time=0, follow_for=follow_for_distance,
        initial_position=initial_position, distance_to_cover=(degrees_for_distance(13)))

    # align with angler artifact
    await pivot_gyro_turn_abs(-100, 100, -99, stop=True)

    # turn motor c to lift angler artifact
    await motor.run_for_degrees(port.C,-300, 400)

    # align with angler artifact
    await pivot_gyro_turn_abs(100, -100, -90, stop=True)

    # go forward to get away from angler artifact
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1, ki=0.0002, kd=0.2, speed=-400, target_angle=-90, sleep_time=0, follow_for=follow_for_distance,
        initial_position=initial_position, distance_to_cover=(degrees_for_distance(2)))

    # turn right to start aligning with what's on sale market ware
    await pivot_gyro_turn_abs(100, -100, -25, stop=True)

    # go forward to get align and latch with what's on sale market wares
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1, ki=0.0002, kd=0.2, speed=-400, target_angle=-25, sleep_time=0, follow_for=follow_for_distance,
        initial_position=initial_position, distance_to_cover=(degrees_for_distance(30)))

    # go backwards to complete what's on sale market ware
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1, ki=-0.0002, kd=-0.2, speed=600, target_angle=-25, sleep_time=0, follow_for=follow_for_distance,
        initial_position=initial_position, distance_to_cover=(degrees_for_distance(15)))

    # turn right to escape what's on sale
    await pivot_gyro_turn_abs(100, -100, 25, stop=True)

    # go backwards to complete what's on sale market ware
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1, ki=0.0002, kd=0.2, speed=-1000, target_angle=20, sleep_time=0, follow_for=follow_for_distance,
        initial_position=initial_position, distance_to_cover=(degrees_for_distance(45)))


async def run_e():
    pass

async def run_f():
    # go forward to get out of base and approach silo
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1, ki=-0.0002, kd=-0.2, speed=650, target_angle=0, sleep_time=0, follow_for=follow_for_distance,
        initial_position=initial_position, distance_to_cover=(degrees_for_distance(37.5)))

    # move hammer down to hit silo lever (1)
    await motor.run_for_degrees(port.C, 340, 1100, acceleration=5000)

    # move up hammer to get ready to hit silo again (1)
    await motor.run_for_degrees(port.C, 340, -1100)

    # move hammer down to hit silo lever (2)
    await motor.run_for_degrees(port.C, 340, 1100, acceleration=5000)

    # move up hammer to get ready to hit silo again (2)
    await motor.run_for_degrees(port.C, 340, -1100)

    # move hammer down to hit silo lever (3)
    await motor.run_for_degrees(port.C, 340, 1100, acceleration=5000)

    # move up hammer to approach who lived here (3)
    await motor.run_for_degrees(port.C, 340, -1100)

    # turn left to apprach who lived here without coliding with forge
    await pivot_gyro_turn_abs(left_speed=-100, right_speed=100, angle=-14, stop=True)

    # go forward to approach who lived here
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1, ki=-0.0002, kd=-0.2, speed=400, target_angle=-14, sleep_time=0, follow_for=follow_for_distance,
    initial_position=initial_position, distance_to_cover=(degrees_for_distance(39)))

    # turn left to complete who lived here
    await pivot_gyro_turn_abs(left_speed=-100, right_speed=100, angle=-28, stop=True)

    # go backwards to ensure correct alignment to release ore blocks
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1, ki=0.0002, kd=0.2, speed=-450, target_angle=-28, sleep_time=0, follow_for=follow_for_distance,
    initial_position=initial_position, distance_to_cover=(degrees_for_distance(11)))

    # turn right to align with forge and release ore blocks
    await pivot_gyro_turn_abs(left_speed=250, right_speed=-250, angle=40, stop=True)

    # bring heavy lifting arm down
    await motor.run_for_degrees(port.B, 2100, -1100)

    # go forward to engage with heavy lifting
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1, ki=-0.0002, kd=-0.2, speed=100, target_angle=40, sleep_time=0, follow_for=follow_for_distance,
    initial_position=initial_position, distance_to_cover=(degrees_for_distance(10)))

    # bring heavy lifting arm up to pick up heavy lifting
    await motor.run_for_degrees(port.B, 800, 1000)
    motor.run_for_degrees(port.B, 1200, 1000)

    # go backwards from forge
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1, ki=0.0002, kd=0.2, speed=-800, target_angle=40, sleep_time=0, follow_for=follow_for_distance,
    initial_position=initial_position, distance_to_cover=(degrees_for_distance(25)))

    # turn left to  align to get back to base 
    await pivot_gyro_turn_abs(left_speed=-800, right_speed=800, angle=-18, stop=True)

    # # go backwards from forge
    # motor.reset_relative_position(port.A, 0)
    # initial_position = abs(motor.relative_position(port.A))
    # await follow_gyro_angle(kp=1, ki=0.0002, kd=0.2, speed=-800, target_angle=40, sleep_time=0, follow_for=follow_for_distance,
    # initial_position=initial_position, distance_to_cover=(degrees_for_distance(25)))

    # # turn right to start aligning to push forge pieces in to base
    # await pivot_gyro_turn_abs(left_speed=100, right_speed=0, angle=125, stop=True)

    # # go backwards to align
    # motor.reset_relative_position(port.A, 0)
    # initial_position = abs(motor.relative_position(port.A))
    # await follow_gyro_angle(kp=1, ki=0.0002, kd=0.2, speed=-800, target_angle=125, sleep_time=0, follow_for=follow_for_distance,
    # initial_position=initial_position, distance_to_cover=(degrees_for_distance(14)))

    # # turn left to fully align to push in ore blocks
    # await pivot_gyro_turn_abs(left_speed=-100, right_speed=100, angle=72, stop=True)

    # # bring heavy lifting arm down to push ore blocks in
    # await motor.run_for_degrees(port.B, 1700, -800)

    # # go forward to push in ore blocks
    # motor.reset_relative_position(port.A, 0)
    # initial_position = abs(motor.relative_position(port.A))
    # await follow_gyro_angle(kp=-1, ki=-0.0002, kd=-0.2, speed=200, target_angle=70, sleep_time=0, follow_for=follow_for_distance,
    # initial_position=initial_position, distance_to_cover=(degrees_for_distance(10)))

    # go back towards the base
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1, ki=0.0002, kd=0.2, speed=-1000, target_angle=-14, sleep_time=0, follow_for=follow_for_distance,
    initial_position=initial_position, distance_to_cover=(degrees_for_distance(74)))


# END RUN FUNCTIONS
#----------------------------------------

#-------------------------------------------------------------------------------------------------------------------------------------------------------------

# MAIN EXECUTE FUNCTION
#----------------------------------------

async def execute(run_numbers=None):

    runs_to_execute = list()

    if isinstance(run_numbers, int):
        run_numbers = [run_numbers]

    # If run_numbers are not provided execute all runs
    runs_to_execute = run_numbers if run_numbers else [2]

    start_times = [time.ticks_ms() for _ in runs_to_execute]
    end_times = [time.ticks_ms() for _ in runs_to_execute]

    run_functions_map = {
                            'a': run_a,
                            'c': run_c,
                            'd': run_d,
                            'e': run_e,
                            'f': run_f,
                        }
    print("Start - Execute")

    # Initialization
    # Define motor pai for robot movements
    motor_pair.pair(motor_pair.PAIR_1, port.A, port.E)

    do_init()
    light_matrix.write("0")
    light.color(light.POWER, color.RED)

    for i, run_number in enumerate(runs_to_execute):

        # waiting for left button to be pressed to start the run
        await runloop.until(is_left_button_pressed)
        print("Starting Run: " + str(run_number))

        light.color(light.POWER, color.MAGENTA)
        light_matrix.show_image(light_matrix.IMAGE_BUTTERFLY)

        start_times[i] = time.ticks_ms()
        do_init()

        runloop.run(run_functions_map[run_number]())
        end_times[i] = time.ticks_ms()
        light.color(light.POWER, color.YELLOW)

        if i > 0:
            print("Transition time: " + str(get_time_taken_in_seconds(end_times[i - 1], start_times[i])) + " s")
        print("Run " + str(run_number) + " time " + str(get_time_taken_in_seconds(start_times[i], end_times[i])) + " s")
        print("---------------------------------------------------------------------------")

    # Print execution times
    print("---------------------------------------------------------------------------")
    print("SUMMARY:")
    total_runs_time = 0
    total_transitions_time = 0
    total_time = 0

    for i, run_number in enumerate(runs_to_execute):
        if i > 0:
            transition_time = get_time_taken_in_seconds(end_times[i - 1], start_times[i])
            print("Transition time: " + str(transition_time) + " s")
            total_transitions_time += transition_time
            total_time += transition_time

        run_time = get_time_taken_in_seconds(start_times[i], end_times[i])
        print("Run " + str(run_number) + " time " + str(run_time) + " s")
        total_runs_time += run_time
        total_time += run_time

    print("***************************************************************************")

    print("TOTAL RUN TIME = " + str(total_runs_time) + " s")
    print("TOTAL TRANSITIONS TIME = " + str(total_transitions_time) + " s")
    print("TOTAL TIME = " + str(total_transitions_time + total_runs_time) + " s")

    print("***************************************************************************")


# END MAIN EXECUTE FUNCTION
#----------------------------------------

# Integrated Runs

# SLOT 0 - All Runs
runloop.run(execute(['a', 'c', 'd', 'e', 'f']))

# SLOT 1 - Run C Onwards
# runloop.run(execute(['c', 'd', 'e', 'f']))

# SLOT 2 - Run D Onwards
# runloop.run(execute(['d', 'e', 'f']))

# SLOT 3 - Run E Onwards
# runloop.run(execute(['e', 'f']))

# SLOT 4 - Run F
# runloop.run(execute(['f']))
