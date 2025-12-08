#!/usr/bin/env python3

import hub
import sys
import time

import color, motor, motor_pair, runloop
from hub import light_matrix, button, motion_sensor, light, port, sound


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
                            brake_action,
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
    motor_pair.stop(motor_pair.PAIR_1, stop=brake_action)


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
async def run_1():
    # Lower topsoil hooks to get in position
    motor.run_for_degrees(port.C, 400, 1100)

    # go forward partially to get out of base and approach Map Reveal
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1, ki=-0.0002, kd=-0.2, speed=800, target_angle=0, sleep_time=0, brake_action=motor.HOLD, follow_for=follow_for_distance,
        initial_position=initial_position, distance_to_cover=(degrees_for_distance(50)))

    # go forward fully slowly to get out of base and approach Map Reveal and Flick Surface brushing #1
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1, ki=-0.0002, kd=-0.2, speed=700, target_angle=0, sleep_time=0, brake_action=motor.HOLD, follow_for=follow_for_distance,
        initial_position=initial_position, distance_to_cover=(degrees_for_distance(22)))

    # Move backward to Flick the surface brushing brush #2
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1, ki=0.0002, kd=0.2, speed=-500, target_angle=0, sleep_time=0, brake_action=motor.HOLD, follow_for=follow_for_distance,
    initial_position=initial_position, distance_to_cover=(degrees_for_distance(17)))

    time.sleep(0.5)

    # Raise Surface Brushing Brush to lift up brush
    await motor.run_for_degrees(port.B, -1200, 800)

    # go forward to aproach map reveal and get ready to turn
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1, ki=-0.0002, kd=-0.2, speed=500, target_angle=0, sleep_time=0, brake_action=motor.HOLD, follow_for=follow_for_distance,
        initial_position=initial_position, distance_to_cover=(degrees_for_distance(19)))

    # raise Topsoil hook to get in position for lifting topsoil piece
    motor.run_for_degrees(port.C, -140, 1100)

    # turn left to get in alignment with Map reveal
    await pivot_gyro_turn_abs(left_speed=-100, right_speed=0, angle=-40, stop=True)

    # go forward partially to get in position to lift topsoil
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1, ki=-0.0002, kd=-0.2, speed=600, target_angle=-40, sleep_time=0, brake_action=motor.HOLD, follow_for=follow_for_distance,
        initial_position=initial_position, distance_to_cover=(degrees_for_distance(1.5)))

    # raise Topsoil hook to lift topsoil piece
    await motor.run_for_degrees(port.C, -275, 300)

    # go forward faster to complete moving Map Reveal piece 1 and 2 to the back
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1, ki=-0.0002, kd=-0.2, speed=300, target_angle=-40, sleep_time=0, brake_action=motor.HOLD, follow_for=follow_for_distance,
        initial_position=initial_position, distance_to_cover=(degrees_for_distance(24.5)))


    # Raise Surface Brushing Brush to to unlock leave-in attachment
    await motor.run_for_degrees(port.B, 500, 800)

    # Move backward to move away from Map reveal
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1, ki=0.0002, kd=0.2, speed=-650, target_angle=-40, sleep_time=0, brake_action=motor.HOLD, follow_for=follow_for_distance,
    initial_position=initial_position, distance_to_cover=(degrees_for_distance(10)))

    # Turn left to go to the base
    await pivot_gyro_turn_abs(left_speed=-200, right_speed=200, angle=-150, stop=True)

    # Drop surface brush in forum
    await motor.run_for_degrees(port.B, 600, 800)

    # go forward to go to the base
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1, ki=-0.0002, kd=-0.2, speed=1100, target_angle=-165, sleep_time=0, brake_action=motor.HOLD, follow_for=follow_for_distance,
        initial_position=initial_position, distance_to_cover=(degrees_for_distance(65)))


async def run_2():
    # Turn right to align with forum
    await pivot_gyro_turn_abs(left_speed=0, right_speed=-200, angle=4, stop=True)

    # Go major distance backwards (fast) to align with the back walls
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1, ki=0.0002, kd=0.2, speed=-850, target_angle=4.5, sleep_time=0, brake_action=motor.HOLD, follow_for=follow_for_distance,
        initial_position=initial_position, distance_to_cover=(degrees_for_distance(75)))

    # Go all the way backwards (slower) to align with the back walls
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1, ki=0.0002, kd=0.2, speed=-300, target_angle=4.5, sleep_time=0, brake_action=motor.HOLD, follow_for=follow_for_distance,
        initial_position=initial_position, distance_to_cover=(degrees_for_distance(15)))

    # Go forward to prepare turning left
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1, ki=-0.0002, kd=-0.2, speed=300, target_angle=0, sleep_time=0, brake_action=motor.BRAKE, follow_for=follow_for_distance,
        initial_position=initial_position, distance_to_cover=(degrees_for_distance(3)))

    # (In Parallel) Lower the arm for mineshaft explorer
    motor.run_for_degrees(port.C, 375, 300)

    # Turn left to face precious-artifact
    await pivot_gyro_turn_abs(left_speed=-100, right_speed=100, angle=-90, stop=True)

    # Go forward to make contact with precious-artifact
    # Using raw movement to avoid gyro interaction
    await motor_pair.move_for_degrees(motor_pair.PAIR_1, degrees_for_distance(5.67), 0, velocity=100)

    # Lower arm to engage with precious artifact
    await motor.run_for_degrees(port.B, -138, 100, stop=motor.HOLD, acceleration=200, deceleration=200)

    # Go forward to make contact with precious-artifact
    await motor_pair.move_for_degrees(motor_pair.PAIR_1, degrees_for_distance(6.5), 0, velocity=50)

    # (In Paralell) Lift arm slightly to lift precious-artifact
    await motor.run_for_degrees(port.B, 180, 200)

    # Lift arm to operate "Mineshaft Explorer"
    await motor.run_for_degrees(port.C, -320, 400)

    # Go backward to snatch the precious artifact and move away from carful recovery
    await motor_pair.move_for_degrees(motor_pair.PAIR_1, degrees_for_distance(15), 0, velocity=-500)

    # Turn right to align with forum
    await pivot_gyro_turn_abs(left_speed=200, right_speed=-200, angle=-50, stop=True)

    # (In Paralell) Lower the arm to drop off precious-artifact
    motor.run_for_degrees(port.B, -300, 300)

    # Go forward to forum for dropping off the precious artifact
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1, ki=-0.0002, kd=-0.2, speed=800, target_angle=-50, sleep_time=0, brake_action=motor.BRAKE, follow_for=follow_for_distance,
        initial_position=initial_position, distance_to_cover=(degrees_for_distance(10)))

    # # Go backwards to get away from forum
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1, ki=0.0002, kd=0.2, speed=-800, target_angle=-50, sleep_time=0, brake_action=motor.HOLD, follow_for=follow_for_distance,
        initial_position=initial_position, distance_to_cover=(degrees_for_distance(4)))

    # Turn right to face the base
    await pivot_gyro_turn_abs(left_speed=200, right_speed=-200, angle=-5, stop=True)

    await pivot_gyro_turn_abs(left_speed=200, right_speed=-200, angle=25, stop=True)

    # Go forward to the base
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1, ki=-0.0002, kd=-0.2, speed=1100, target_angle=25, sleep_time=0, brake_action=motor.BRAKE, follow_for=follow_for_distance,
        initial_position=initial_position, distance_to_cover=(degrees_for_distance(70)))


async def run_3():
    # go forward to get out of base and approach salvage operation
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1, ki=-0.0002, kd=-0.2, speed=500, target_angle=1, sleep_time=0, brake_action=motor.HOLD, follow_for=follow_for_distance,
        initial_position=initial_position, distance_to_cover=(degrees_for_distance(33)))

    # go forward to approach salvage operation slower
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-2, ki=-0.0002, kd=-0.2, speed=400, target_angle=1, sleep_time=0, brake_action=motor.HOLD, follow_for=follow_for_distance,
        initial_position=initial_position, distance_to_cover=(degrees_for_distance(8)))

    # go forward to approach salvage operation faster
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-2, ki=-0.0002, kd=-0.2, speed=600, target_angle=1, sleep_time=0, brake_action=motor.HOLD, follow_for=follow_for_distance,
        initial_position=initial_position, distance_to_cover=(degrees_for_distance(10)))

    # move arm down to drop flag inside salvage operation
    await motor.run_for_degrees(port.C, 250, 500)

    await sound.beep(duration=500)

    # go back to base slower
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1, ki=0.0002, kd=0.2, speed=-200, target_angle=1, sleep_time=0, brake_action=motor.HOLD, follow_for=follow_for_distance,
        initial_position=initial_position, distance_to_cover=(degrees_for_distance(11)))

    # move flag arm to complete
    motor.run_for_degrees(port.C, -250, 500)

    # go back to base faster
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1, ki=0.0002, kd=0.2, speed=-800, target_angle=1, sleep_time=0, brake_action=motor.BRAKE, follow_for=follow_for_distance,
        initial_position=initial_position, distance_to_cover=(degrees_for_distance(40)))


async def run_4():

        # bring arm down to to start engaging with statue rebuild
    motor.run_for_degrees(port.B, -2300, 1100)

    # turn left to avoid salvage operation
    await pivot_gyro_turn_abs(-200, 0, -20, stop=True)

    # go forward to approach statue rebuild
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1, ki=0.0002, kd=0.2, speed=-600, target_angle=-20, sleep_time=0, brake_action=motor.HOLD, follow_for=follow_for_distance,
        initial_position=initial_position, distance_to_cover=(degrees_for_distance(15)))

    # turn right to align with statue rebuild
    await pivot_gyro_turn_abs(200, -200, 133, stop=True)

    # go forward to statue rebuild
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1, ki=-0.0002, kd=-0.2, speed=600, target_angle=133, sleep_time=0, brake_action=motor.HOLD, follow_for=follow_for_distance,
        initial_position=initial_position, distance_to_cover=(degrees_for_distance(27.5)))

    # bring arm down to to start engaging with statue rebuild
    await motor.run_for_degrees(port.B, -650, 1100)

    # turn right to get lever under statue rebuild
    await pivot_gyro_turn_abs(75, -75, 141, stop=True)

    # wait to make sure the attachment is latched under statue rebuild
    # runloop.sleep_ms(100)
    await hub.sound.beep(duration=100)

    # bring arm up to lift the statue
    await motor.run_for_degrees(port.B, 1200, 1100)

    # go forward to statue rebuild
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1, ki=-0.0002, kd=-0.2, speed=75, target_angle=141, sleep_time=0, brake_action=motor.HOLD, follow_for=follow_for_distance,
        initial_position=initial_position, distance_to_cover=(degrees_for_distance(3)))

    # go backward to move away from statue rebuild
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1, ki=0.0002, kd=0.2, speed=-500, target_angle=141, sleep_time=0, brake_action=motor.HOLD, follow_for=follow_for_distance,
        initial_position=initial_position, distance_to_cover=(degrees_for_distance(12)))

    # bring arm up to lift the statue
    motor.run_for_degrees(port.B, 1800, 1100)

    # turn right to start approaching tip the scale
    await pivot_gyro_turn_abs(-150, 150, 0, stop=True)

    # go backward to start aligning with tip the scale
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1, ki=0.0002, kd=0.2, speed=-700, target_angle=0, sleep_time=0, brake_action=motor.HOLD, follow_for=follow_for_distance,
        initial_position=initial_position, distance_to_cover=(degrees_for_distance(37)))

    # go backward to start aligning with tip the scale
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1, ki=0.0002, kd=0.2, speed=-700, target_angle=5, sleep_time=0, brake_action=motor.HOLD, follow_for=follow_for_distance,
        initial_position=initial_position, distance_to_cover=(degrees_for_distance(38.5)))

    # turn left to start aligning with tip the scales
    await pivot_gyro_turn_abs(-150, 150, -86, stop=True)

    # go forward to get align and latch with tip the scale
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1, ki=0.0002, kd=0.2, speed=-200, target_angle=-86, sleep_time=0, brake_action=motor.HOLD, follow_for=follow_for_distance,
        initial_position=initial_position, distance_to_cover=(degrees_for_distance(14.5)))

    # go backward to go away from tip the scale and pull the pan
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1, ki=-0.0002, kd=-0.2, speed=400, target_angle=-95, sleep_time=0, brake_action=motor.HOLD, follow_for=follow_for_distance,
        initial_position=initial_position, distance_to_cover=(degrees_for_distance(12)))

    # align with angler artifact
    await pivot_gyro_turn_abs(-100, 100, -101, stop=True)

    # turn motor c to lift angler artifact
    await motor.run_for_degrees(port.C, -500, 500)

    # turn to un-latch with angler artifact gear
    await pivot_gyro_turn_abs(100, -100, -90, stop=True)

    # go forward to get away from angler artifact
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1, ki=0.0002, kd=0.2, speed=-400, target_angle=-90, sleep_time=0, brake_action=motor.HOLD, follow_for=follow_for_distance,
        initial_position=initial_position, distance_to_cover=(degrees_for_distance(2)))

    # turn right to start aligning with what's on sale market ware
    await pivot_gyro_turn_abs(150, -150, -18, stop=True)

    # go forward to get align and latch with what's on sale market wares
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1, ki=0.0002, kd=0.2, speed=-600, target_angle=-20, sleep_time=0, brake_action=motor.HOLD, follow_for=follow_for_distance,
        initial_position=initial_position, distance_to_cover=(degrees_for_distance(30)))

    # go backwards to complete what's on sale market ware
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1, ki=-0.0002, kd=-0.2, speed=600, target_angle=-20, sleep_time=0, brake_action=motor.HOLD, follow_for=follow_for_distance,
        initial_position=initial_position, distance_to_cover=(degrees_for_distance(15)))

    # turn right to escape what's on sale
    await pivot_gyro_turn_abs(200, -200, 15, stop=True)

    # go backwards to get to base
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1, ki=0.0002, kd=0.2, speed=-1100, target_angle=15, sleep_time=0, brake_action=motor.BRAKE, follow_for=follow_for_distance,
        initial_position=initial_position, distance_to_cover=(degrees_for_distance(60)))


async def run_5():
    # go forward to get out of base and approach silo
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1, ki=-0.0002, kd=-0.2, speed=650, target_angle=0, sleep_time=0, brake_action=motor.HOLD, follow_for=follow_for_distance,
        initial_position=initial_position, distance_to_cover=(degrees_for_distance(40)))

    # bring arm down to hit silo
    for i in range (0, 3):
        i=i+1
        # move hammer down to hit silo lever (1)
        await motor.run_for_degrees(port.C, 225, 900, acceleration=7000)

        time.sleep_ms(250)

        # move up hammer to get ready to hit silo again (1)
        await motor.run_for_degrees(port.C, 225, -900)

    # bring heavy lifting arm down (1)
    motor.run_for_degrees(port.B, 1700, -1100)

    # turn left to apprach who lived here without coliding with forge
    await pivot_gyro_turn_abs(left_speed=-50, right_speed=50, angle=-13, stop=True)

    # go forward to approach who lived here
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1, ki=-0.0002, kd=-0.2, speed=400, target_angle=-13, sleep_time=0, brake_action=motor.HOLD, follow_for=follow_for_distance,
    initial_position=initial_position, distance_to_cover=(degrees_for_distance(34)))

    # turn left to complete who lived here
    await pivot_gyro_turn_abs(left_speed=-200, right_speed=200, angle=-30, stop=True)

    # go backwards to ensure correct alignment to release ore blocks
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1, ki=0.0002, kd=0.2, speed=-450, target_angle=-30, sleep_time=0, brake_action=motor.HOLD, follow_for=follow_for_distance,
    initial_position=initial_position, distance_to_cover=(degrees_for_distance(9)))

    # turn right to align with forge and release ore blocks
    await pivot_gyro_turn_abs(left_speed=250, right_speed=-250, angle=45, stop=True)

    # bring heavy lifting arm down (2)
    await motor.run_for_degrees(port.B, 550, -1100)

    # go forward to engage with heavy lifting
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1, ki=-0.0002, kd=-0.2, speed=100, target_angle=40, sleep_time=0, brake_action=motor.HOLD, follow_for=follow_for_distance,
    initial_position=initial_position, distance_to_cover=(degrees_for_distance(10)))

    # bring heavy lifting arm up to pick up heavy lifting
    await motor.run_for_degrees(port.B, 900, 1000)
    motor.run_for_degrees(port.B, 1300, 1000)

    # go backwards from forge
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1, ki=0.0002, kd=0.2, speed=-800, target_angle=40, sleep_time=0, brake_action=motor.HOLD, follow_for=follow_for_distance,
    initial_position=initial_position, distance_to_cover=(degrees_for_distance(23)))

    # turn left to align to get back to base
    await pivot_gyro_turn_abs(left_speed=-800, right_speed=800, angle=-18, stop=True)

    # go back towards the base
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1, ki=0.0002, kd=0.2, speed=-1000, target_angle=-18, sleep_time=0, brake_action=motor.BRAKE, follow_for=follow_for_distance,
    initial_position=initial_position, distance_to_cover=(degrees_for_distance(75)))


async def run_6():
    # turn left to escape what's on sale
    await pivot_gyro_turn_abs(0, 100, -25, stop=True)

    # go forward to align with opposing mineshaft explorer
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1, ki=-0.0002, kd=-0.2, speed=800, target_angle=-25, sleep_time=0, brake_action=motor.HOLD, follow_for=follow_for_distance,
        initial_position=initial_position, distance_to_cover=(degrees_for_distance(68)))

    # turn left to escape what's on sale
    await pivot_gyro_turn_abs(-100, 100, -35, stop=True)

    # go forward to align with opposing mineshaft explorer
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1, ki=-0.0002, kd=-0.2, speed=700, target_angle=-35, sleep_time=0, brake_action=motor.HOLD, follow_for=follow_for_distance,
        initial_position=initial_position, distance_to_cover=(degrees_for_distance(5)))

    # turn left to align with flag dropoff
    await pivot_gyro_turn_abs(-100, 100, -88, stop=True)

    # go forward to align with opposing mineshaft explorer
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1, ki=-0.0002, kd=-0.2, speed=500, target_angle=-90, sleep_time=0, brake_action=motor.HOLD, follow_for=follow_for_distance,
        initial_position=initial_position, distance_to_cover=(degrees_for_distance(28)))

    # turn right to align with flag dropoff
    await pivot_gyro_turn_abs(100, -100, 0, stop=True)

    # go forward to drop off the flag
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1, ki=-0.0002, kd=-0.2, speed=300, target_angle=0, sleep_time=0, brake_action=motor.HOLD, follow_for=follow_for_distance,
        initial_position=initial_position, distance_to_cover=(degrees_for_distance(8)))

    # lift opposing team mineshaft
    await motor.run_for_degrees(port.C, 1000, 1000)

    # go backward to leave flag
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1, ki=0.0002, kd=0.2, speed=-300, target_angle=0, sleep_time=0, brake_action=motor.HOLD, follow_for=follow_for_distance,
        initial_position=initial_position, distance_to_cover=(degrees_for_distance(7)))

    # turn left to align with whats on sale
    await pivot_gyro_turn_abs(-100, 100, -45, stop=True)

    # go backward to push the roof for whats on sale
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1, ki=0.0002, kd=0.2, speed=-500, target_angle=-45, sleep_time=0, brake_action=motor.HOLD, follow_for=follow_for_distance,
        initial_position=initial_position, distance_to_cover=(degrees_for_distance(20)))

    # go backward to push the roof for whats on sale
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1, ki=0.0002, kd=0.2, speed=-700, target_angle=-45, sleep_time=0, brake_action=motor.HOLD, follow_for=follow_for_distance,
        initial_position=initial_position, distance_to_cover=(degrees_for_distance(15)))

    # go forward to start aligning with forum
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1, ki=-0.0002, kd=-0.2, speed=300, target_angle=-45, sleep_time=0, brake_action=motor.HOLD, follow_for=follow_for_distance,
    initial_position=initial_position, distance_to_cover=(degrees_for_distance(11)))

    # turn left to start aligning with forum
    await pivot_gyro_turn_abs(-100, 100, -90, stop=True)

    # go forward to start aligning with forum
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1, ki=-0.0002, kd=-0.2, speed=700, target_angle=-90, sleep_time=0, brake_action=motor.HOLD, follow_for=follow_for_distance,
        initial_position=initial_position, distance_to_cover=(degrees_for_distance(30)))

    # turn left to start aligning with forum
    await pivot_gyro_turn_abs(-100, 100, -105, stop=True)

    # go forward to start aligning with forum
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1, ki=-0.0002, kd=-0.2, speed=700, target_angle=-105, sleep_time=0, brake_action=motor.HOLD, follow_for=follow_for_distance,
        initial_position=initial_position, distance_to_cover=(degrees_for_distance(19)))

    # turn left to start aligning with forum
    await pivot_gyro_turn_abs(-100, 100, -150, stop=True)

    # go forward to drop pieces in to forum
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1, ki=-0.0002, kd=-0.2, speed=200, target_angle=-150, sleep_time=0, brake_action=motor.HOLD, follow_for=follow_for_distance,
        initial_position=initial_position, distance_to_cover=(degrees_for_distance(4)))

    # turn left to start aligning with forum
    await pivot_gyro_turn_abs(-100, 100, -160, stop=True)

    # go forward to drop pieces in to forum
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1, ki=-0.0002, kd=-0.2, speed=200, target_angle=-160, sleep_time=0, brake_action=motor.HOLD, follow_for=follow_for_distance,
        initial_position=initial_position, distance_to_cover=(degrees_for_distance(2)))

    # drop off opposing team mineshaft in forum
    motor.run_for_degrees(port.C, -2000, 1100)

    #turn motor b to drop scale pan and heavy lifting onto forum
    await motor.run_for_degrees(port.B, -1800, 1100)

    #turn motor b to release scale pan and heavy lifting onto forum
    await motor.run_for_degrees(port.B, 600, 1100)

    # go backwards from forum
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1, ki=0.0002, kd=0.2, speed=-400, target_angle=-160, sleep_time=0, brake_action=motor.HOLD, follow_for=follow_for_distance,
        initial_position=initial_position, distance_to_cover=(degrees_for_distance(5)))

    # turn left to start aligning with forum
    await pivot_gyro_turn_abs(100, -100, -90, stop=True)

    # go forward to start aligning with forum
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1, ki=-0.0002, kd=-0.2, speed=700, target_angle=-90, sleep_time=0, brake_action=motor.HOLD, follow_for=follow_for_distance,
        initial_position=initial_position, distance_to_cover=(degrees_for_distance(15)))


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
                            1: run_1,
                            2: run_2,
                            3: run_3,
                            4: run_4,
                            5: run_5,
                            6: run_6,
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
runloop.run(execute([1, 2, 3, 4, 5, 6]))

# SLOT 1 - Run 2 Onwards
# runloop.run(execute([2, 3, 4, 5, 6]))

# SLOT 2 - Run 3 Onwards
# runloop.run(execute([3, 4, 5, 6]))

# SLOT 3 - Run 4 Onwards
# runloop.run(execute([4, 5, 6]))

# SLOT 4 - Run 5 Onwards
# runloop.run(execute([5, 6]))

# SLOT 5 - Run 6
# runloop.run(execute([6]))
