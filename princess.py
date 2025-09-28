#!/usr/bin/env python3

import hub
import sys
import time

import color, motor, motor_pair, runloop
from hub import light_matrix, button, motion_sensor, light, port


# CONSTANTS
#----------------------------------------

WHEEL_CIRCUMFERENCE = 17.584

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

# run 1 program
async def run1():

    # go backward to get out of base
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.45, ki=0, kd=0, speed=-500, target_angle=0, sleep_time=0, follow_for=follow_for_distance,
            initial_position=initial_position, distance_to_cover=(degrees_for_distance(3)))

    # turn left to get in alignment with krill
    await pivot_gyro_turn_abs(left_speed=-200, right_speed=200, angle=-45, stop=True)

    # go backward to collect krill
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.45, ki=0, kd=0, speed=-600, target_angle=-45, sleep_time=0, follow_for=follow_for_distance,
            initial_position=initial_position, distance_to_cover=(degrees_for_distance(16)))

    # turn right to get in alignment with coral piece
    await pivot_gyro_turn_abs(left_speed=200, right_speed=-200, angle=0, stop=True)

    # go backward to collect coral piece
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.45, ki=0, kd=0, speed=-600, target_angle=0, sleep_time=0, follow_for=follow_for_distance,
            initial_position=initial_position, distance_to_cover=(degrees_for_distance(35)))

    # go forward to leave pieces for shipping lanes
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.45, ki=0, kd=0, speed=600, target_angle=0, sleep_time=0, follow_for=follow_for_distance,
            initial_position=initial_position, distance_to_cover=(degrees_for_distance(17)))

    # turn right to get in alignment with changing shipping lanes
    await pivot_gyro_turn_abs(left_speed=150, right_speed=-150, angle=45, stop=True)

    # go backward to engage with shipping lanes
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.45, ki=0, kd=0, speed=-400, target_angle=45, sleep_time=0, follow_for=follow_for_distance,
            initial_position=initial_position, distance_to_cover=(degrees_for_distance(16)))

    # raise shipping lane/seabed attachment to lift shipping lanes
    await motor.run_for_degrees(port.C, 1000, 1100)

    # turn right to drop shipping lanes on other side
    await pivot_gyro_turn_abs(left_speed=125, right_speed=-125, angle=100, stop=True)

    # go forward to leave shipping lanes
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.45, ki=0, kd=0, speed=400, target_angle=100, sleep_time=0, follow_for=follow_for_distance,
            initial_position=initial_position, distance_to_cover=(degrees_for_distance(12.5)))

    # turn left to get back into alignment with krill/coral pieces
    await pivot_gyro_turn_abs(left_speed=-150, right_speed=150, angle=4, stop=True)

    # reset shipping lanes attachment to get ready for sample collection
    motor.run_for_degrees(port.C, -500, 900)

    # go backward to recollect pieces
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.45, ki=0, kd=0, speed=-600, target_angle=0, sleep_time=0, follow_for=follow_for_distance,
            initial_position=initial_position, distance_to_cover=(degrees_for_distance(18)))

    # turn right to align with last krill
    await pivot_gyro_turn_abs(left_speed=200, right_speed=-200, angle=45, stop=True)

    # go backward to collect last krill
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.45, ki=0, kd=0, speed=-400, target_angle=45, sleep_time=0, follow_for=follow_for_distance,
            initial_position=initial_position, distance_to_cover=(degrees_for_distance(5.5)))

    # turn to align with plankton hook
    # await pivot_gyro_turn_abs(left_speed=200, right_speed=-200, angle=165, stop=True)
    await pivot_gyro_turn_abs(left_speed=-125, right_speed=125, angle=-87, stop=True)

    # go forward to hook into plankton
    await motor_pair.move_for_degrees(motor_pair.PAIR_1, degrees_for_distance(12),0,velocity=300)

    # go backward to pull plankton
    await motor_pair.move_for_degrees(motor_pair.PAIR_1, degrees_for_distance(2),0,velocity=-300)

    # go forward to get away from sonar discovery
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.45, ki=0, kd=0, speed=-500, target_angle=-88, sleep_time=0, follow_for=follow_for_distance,
            initial_position=initial_position, distance_to_cover=(degrees_for_distance(5)))

    # turn right to go forward
    await pivot_gyro_turn_abs(left_speed=-150, right_speed=150, angle=-91, stop=True)

    # go backward toward seabed
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.45, ki=0, kd=0, speed=-1000, target_angle=-91, sleep_time=0, follow_for=follow_for_distance,
            initial_position=initial_position, distance_to_cover=(degrees_for_distance(19)))

    # go backward toward seabed
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.45, ki=0, kd=0, speed=-1000, target_angle=-90, sleep_time=0, follow_for=follow_for_distance,
            initial_position=initial_position, distance_to_cover=(degrees_for_distance(77)))

    # bring send over the submersible attachment down
    motor.run_for_degrees(port.B, 2500, 1100)

    # go forward (back) to leave pieces
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.45, ki=0, kd=0, speed=500, target_angle=-94, sleep_time=0, follow_for=follow_for_distance,
            initial_position=initial_position, distance_to_cover=(degrees_for_distance(16)))

    # turn to align to seabed sample
    await pivot_gyro_turn_abs(left_speed=300, right_speed=-300, angle=0, stop=True)
    await turnRight(8)

    # go forward to engage with seabed sample
    await motor_pair.move_for_degrees(motor_pair.PAIR_1, degrees_for_distance(17),0,velocity=-400)

    # raise seabed sample hook to raise the sample and collect it
    # raise send over the submersible attachment
    motor.run_for_degrees(port.C, 1000, 900)
    await motor.run_for_degrees(port.B, 1300, -1000)
    await runloop.sleep_ms(500)
    motor.run_for_degrees(port.C, 700, 900)

    # come back from seabed
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.45, ki=0, kd=0, speed=400, target_angle=0, sleep_time=0, follow_for=follow_for_distance,
            initial_position=initial_position, distance_to_cover=(degrees_for_distance(10)))

    # turn to leave seabed sample
    await pivot_gyro_turn_abs(left_speed=-200, right_speed=200, angle=-93, stop=True)

    # go forward to recollect samples
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.45, ki=0, kd=0, speed=-700, target_angle=-93, sleep_time=0, follow_for=follow_for_distance,
    initial_position=initial_position, distance_to_cover=(degrees_for_distance(19)))

    # turn left to align with water sample/krill
    await pivot_gyro_turn_abs(left_speed=-200, right_speed=200, angle=-101, stop=True)

    # go forward to collect water sample and krill
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.45, ki=0, kd=0, speed=-500, target_angle=-101, sleep_time=0, follow_for=follow_for_distance,
    initial_position=initial_position, distance_to_cover=(degrees_for_distance(16)))

    # turn left collect coral piece
    await pivot_gyro_turn_abs(left_speed=-200, right_speed=200, angle=-108, stop=True)

    # go backward to collect coral piece
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.45, ki=0, kd=0, speed=-400, target_angle=-108, sleep_time=0, follow_for=follow_for_distance,
    initial_position=initial_position, distance_to_cover=(degrees_for_distance(16)))

    # turn left to collect last coral piece
    await pivot_gyro_turn_abs(left_speed=-200, right_speed=200, angle=-170, stop=True)

    # go forward to collect last coral piece
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.45, ki=0, kd=0, speed=-1100, target_angle=-170, sleep_time=0, follow_for=follow_for_distance,
    initial_position=initial_position, distance_to_cover=(degrees_for_distance(30)))

    # go forward to get into base
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.45, ki=0, kd=0, speed=-1100, target_angle=-140, sleep_time=0, follow_for=follow_for_distance,
    initial_position=initial_position, distance_to_cover=(degrees_for_distance(45)))

# END RUN 1
#----------------------------------------

# RUN 2
#----------------------------------------
# run 2 program - Raise the mast, Kraken's treasure, Diver Pickup, Diver Drop off, Coral buds, Coral Reef Buds, Shark Pick up, Coral Tree
async def run2():

    # go straight to get out of base
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.45, ki=0, kd=0, speed=600, target_angle=0, sleep_time=0, follow_for=follow_for_distance,
                    initial_position=initial_position, distance_to_cover=(degrees_for_distance(15)))

    # turn left to get out of base
    await pivot_gyro_turn_abs(left_speed=-250, right_speed=250, angle=-145, stop=True)

    # go straight (backward) to align with shipwreck
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.45, ki=0, kd=0, speed=-650, target_angle=-145, sleep_time=0, follow_for=follow_for_distance,
                    initial_position=initial_position, distance_to_cover=(degrees_for_distance(37)))

    # turn right to get in front of shipwreck
    await pivot_gyro_turn_abs(left_speed=100, right_speed=-100, angle=-90, stop=True)

    # go straight to engage with shipwreck
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.45, ki=0, kd=0, speed=-350, target_angle=-90, sleep_time=0, follow_for=follow_for_distance,
                    initial_position=initial_position, distance_to_cover=(degrees_for_distance(24.5)))

    # lower fork arm to get in position to pick up diver
    motor.run_for_degrees(port.C, -500, 1100)

    # come back to collect treasure and release mast
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.45, ki=0, kd=0, speed=450, target_angle=-90, sleep_time=0, follow_for=follow_for_distance,
                    initial_position=initial_position, distance_to_cover=(degrees_for_distance(23)))

    # turn left to prepare for alignment with coral tree
    await pivot_gyro_turn_abs(left_speed=-100, right_speed=100, angle=-145, stop=True)

    # lower fork arm to get in position to pick up diver
    motor.run_for_degrees(port.C, -515, 1100)

    # go forward to prepare for alignment with coral tree
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.45, ki=0, kd=0, speed=-500, target_angle=-145, sleep_time=0, follow_for=follow_for_distance,
                    initial_position=initial_position, distance_to_cover=(degrees_for_distance(13)))

    # turn right to get in alignment with coral tree
    await pivot_gyro_turn_abs(left_speed=100, right_speed=-100, angle=-89, stop=True)

    # lower coral tree arm to get in postion to lift coral tree
    motor.run_for_degrees(port.B, 175, 250)

    # lower fork arm to get in position to pick up diver
    await motor.run_for_degrees(port.C, -1100, 1100)

    # go straight to push coral tree buds
    await motor_pair.move_for_degrees(motor_pair.PAIR_1, degrees_for_distance(14), 0, velocity=200)

    # lift coral tree arm to complete coral tree mission
    await motor.run_for_degrees(port.B, -75, 150)
    await motor.run_for_degrees(port.B, -100, 300)

    # raise fork arm to pick up diver
    await motor.run_for_degrees(port.C, 400, 500)

    # bring coral tree arm down
    await motor.run_for_degrees(port.B, 175, 250)

    # raise fork arm to pick up diver (2)
    motor.run_for_degrees(port.C, 800, 1100)

    # come back from coral tree to get in alignment with Scuba Diver
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.45, ki=0, kd=0, speed=-500, target_angle=-90, sleep_time=0, follow_for=follow_for_distance,
                    initial_position=initial_position, distance_to_cover=(degrees_for_distance(8)))

    # bring coral tree arm up
    await motor.run_for_degrees(port.B, -175, 200)

    # turn right to get in alignment with scuba diver
    await pivot_gyro_turn_abs(left_speed=150, right_speed=-150, angle=0, stop=True)
    await pivot_gyro_turn_abs(left_speed=150, right_speed=-150, angle=13, stop=True)

    # come back to get in alignment with scuba diver
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.45, ki=0, kd=0, speed=-400, target_angle=13, sleep_time=0, follow_for=follow_for_distance,
                    initial_position=initial_position, distance_to_cover=(degrees_for_distance(5.5)))

    # put fork down to drop off Scuba Diver
    await motor.run_for_degrees(port.C, -925, 1100)

    # go forward towards scuba diver drop off
    await motor_pair.move_for_degrees(motor_pair.PAIR_1, degrees_for_distance(10), 0, velocity=200)

    # move fork down to fully release Scuba Diver
    await motor.run_for_degrees(port.C, -325, 1100)

    # come back and get ready to align with coral reef buds
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.45, ki=0, kd=0, speed=-400, target_angle=12, sleep_time=0, follow_for=follow_for_distance,
                    initial_position=initial_position, distance_to_cover=(degrees_for_distance(8)))

    # turn left to get in alignment with coral reef buds
    await pivot_gyro_turn_abs(left_speed=-150, right_speed=150, angle=6, stop=True)

    # raise fork arm so coral reef hook can engage with yellow lever
    await motor.run_for_degrees(port.C, 150, 1100)

    # go forward to complete coral reef buds mission
    await motor_pair.move_for_degrees(motor_pair.PAIR_1, degrees_for_distance(12.5), 0, velocity=200)

    # lower Shark Hook to push the shark misson lever
    motor.run_for_degrees(port.B, 200, 600)

    # lower fork arm to ensure that the coral buds are pushed down
    await motor.run_for_degrees(port.C, -875, 1100)

    # raise Shark Hook so it does not interfere with any other missions
    await motor.run_for_degrees(port.B, -200, 200)

    # raise fork arm to make sure that it doesen't get stuck when we come back
    await motor.run_for_degrees(port.C, 775, 1100)
    motor.run_for_degrees(port.C, 995, 1100)

    # come back a bit to get to base
    await motor_pair.move_for_degrees(motor_pair.PAIR_1, degrees_for_distance(53), 0, velocity=-1100)

    # raise fork arm to ensurre it isn't out of base
    motor.run_for_degrees(port.C, 1400, 1100)

    # turn right to get fully in to base
    await pivot_gyro_turn_abs(left_speed=150, right_speed=-150, angle=69, stop=True)

    # come back to get to base
    await motor_pair.move_for_degrees(motor_pair.PAIR_1, degrees_for_distance(50), 0, velocity=-1100)

# END RUN 2
#----------------------------------------

# RUN 3
#----------------------------------------
# run 3 program
async def run3():

    # turn to get ready to align with krill pick up
    await pivot_gyro_turn_abs(left_speed=225, right_speed=0, angle=58, stop=True)

    # bring arm down (1) to save time
    motor.run_for_degrees(port.C, 1200, 1000)

    # move forward to get ready to align with krill pick up
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.45, ki=0, kd=0, speed=600, target_angle=58, sleep_time=0, follow_for=follow_for_distance,
                    initial_position=initial_position, distance_to_cover=(degrees_for_distance(48)))

    # turn to align to pick up of krill
    await pivot_gyro_turn_abs(left_speed=0, right_speed=70, angle=57, stop=True)

    # move forward to align with the shark drop off and krill pick up
    await motor_pair.move_for_degrees(motor_pair.PAIR_1, degrees_for_distance(18.5), 0, velocity=300)

    # move trident hook to latch on to trident
    await motor.run_for_degrees(port.B, 300, -600)

    # move back to drop off shark
    await motor_pair.move_for_degrees(motor_pair.PAIR_1, degrees_for_distance(23.5), 0, velocity=-400)

    # turn to align with ship
    await pivot_gyro_turn_abs(left_speed=200, right_speed=0, angle=90, stop=True)

    # move back to align with the ship
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.45, ki=0, kd=0, speed=-550, target_angle=90, sleep_time=0, follow_for=follow_for_distance,
                    initial_position=initial_position, distance_to_cover=(degrees_for_distance(7)))

    # bring arm down (2) to engage with research vessel
    await motor.run_for_degrees(port.C, 1100, 1000)

    # go forward with boat to get in the docking area
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-2.5, ki=0, kd=0, speed=800, target_angle=88, sleep_time=0, follow_for=follow_for_distance,
                    initial_position=initial_position, distance_to_cover=(degrees_for_distance(54)))
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-2.5, ki=0, kd=0, speed=400, target_angle=88, sleep_time=0, follow_for=follow_for_distance,
                    initial_position=initial_position, distance_to_cover=(degrees_for_distance(22)))

    # come back to ensure arm dosen't get stuck
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.45, ki=0, kd=0, speed=-700, target_angle=88, sleep_time=0, follow_for=follow_for_distance,
                    initial_position=initial_position, distance_to_cover=(degrees_for_distance(9.5)))

    # raise arm so it doesn't get in the way
    await motor.run_for_degrees(port.C, -1000, 1000)
    motor.run_for_degrees(port.C, -1000, 1450)

    # go forward to leave ship and get in alignment with unexpected encounter
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.45, ki=0, kd=0, speed=900, target_angle=88, sleep_time=0, follow_for=follow_for_distance,
                    initial_position=initial_position, distance_to_cover=(degrees_for_distance(41)))

    # turn to align with unexpected encounter
    await pivot_gyro_turn_abs(left_speed=250, right_speed=-250, angle=135, stop=True)

    # go forward (back) to push unexpected encounter lever and catch creature
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.45, ki=0, kd=0, speed=-1000, target_angle=135, sleep_time=0, follow_for=follow_for_distance,
                    initial_position=initial_position, distance_to_cover=(degrees_for_distance(18)))

    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.45, ki=0, kd=0, speed=-300, target_angle=135, sleep_time=0, follow_for=follow_for_distance,
                    initial_position=initial_position, distance_to_cover=(degrees_for_distance(18)))

    # go back (forward) to base
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.45, ki=0, kd=0, speed=1000, target_angle=135, sleep_time=0, follow_for=follow_for_distance,
                    initial_position=initial_position, distance_to_cover=(degrees_for_distance(40)))

# END RUN 3
#----------------------------------------

# RUN 4
#----------------------------------------
# run 4 program
async def run4():

    # go forward to to get out of base and go towards feed the whale fast
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.45, ki=0, kd=0, speed=800, target_angle=0, sleep_time=0, follow_for=follow_for_distance,
                    initial_position=initial_position, distance_to_cover=(degrees_for_distance(35)))

    # go forward to to get out of base and go towards feed the whale slow
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.45, ki=0, kd=0, speed=400, target_angle=0, sleep_time=0, follow_for=follow_for_distance,
                    initial_position=initial_position, distance_to_cover=(degrees_for_distance(7)))

    # turn to avoid shipping lanes
    await pivot_gyro_turn_abs(left_speed=-200, right_speed=200, angle=-26, stop=True)

    # go forward to avoid changing shipping lanes
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.45, ki=0, kd=0, speed=400, target_angle=-26, sleep_time=0, follow_for=follow_for_distance,
                    initial_position=initial_position, distance_to_cover=(degrees_for_distance(23)))

    # turn right to go straight
    await pivot_gyro_turn_abs(left_speed=150, right_speed=-150, angle=0, stop=True)

    # go forward slower to align with sonar discovery
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.45, ki=0, kd=0, speed=600, target_angle=0, sleep_time=0, follow_for=follow_for_distance,
                    initial_position=initial_position, distance_to_cover=(degrees_for_distance(27)))

    # turn Sonar Discovery attachment motor to complete Sonar Discovery
    await motor.run_for_degrees(port.B, -350, -300)

    # turn Sonar Discovery attachment motor not get stuck in Sonar Discovery
    await motor.run_for_degrees(port.B, 100, -300)

    # go backward slower to start aligning with feed the whale
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.45, ki=0, kd=0, speed=-600, target_angle=0, sleep_time=0, follow_for=follow_for_distance,
                    initial_position=initial_position, distance_to_cover=(degrees_for_distance(13)))

    # turn right to align with feed the whale
    await pivot_gyro_turn_abs(100, -100, 35, True)

    # move forward to open whale's mouth
    await motor_pair.move_for_degrees(motor_pair.PAIR_1, degrees_for_distance(21), 0, velocity=600)

    # turn motor to move food tray down
    await motor.run_for_degrees(port.C, 1450, 1100)

    # move motor to lift food tray so it does not make whale vomit while coming back
    await motor.run_for_degrees(port.C, 250, -1100)

    # move robot backward to move away from feed the whale
    await motor_pair.move_for_degrees(motor_pair.PAIR_1, degrees_for_distance(20), 0, velocity=-900)

    # turn left to align with base
    await pivot_gyro_turn_abs(left_speed=-300, right_speed=300, angle=-7, stop=True)

    # move robot backward to get to base
    await motor_pair.move_for_degrees(motor_pair.PAIR_1, degrees_for_distance(80), 0, velocity=-1100)

# END RUN 4
#----------------------------------------


# RUN 5
#----------------------------------------
# run 5 program
async def run5():

    # move forward to get out of base
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.45, ki=0, kd=0, speed=1000, target_angle=0, sleep_time=0, follow_for=follow_for_distance,
        initial_position=initial_position, distance_to_cover=degrees_for_distance(36))

    # move forward at a lower speed for precision
    motor.reset_relative_position(port.A, 0)
    await follow_gyro_angle(kp=-1.45, ki=0, kd=0, speed=200, target_angle=0, sleep_time=0, follow_for=follow_for_distance,
        initial_position=initial_position, distance_to_cover=degrees_for_distance(5.5))

    # turn left a bit to get the momentum for flicking artifical habitat
    await pivot_gyro_turn_abs(-400, 400, -15, True)

    # turn right to flick artificial habitat
    await pivot_gyro_turn_abs(800, -800, 50, True)

    # turn left to get back in alignment with Artifical Habitat
    await pivot_gyro_turn_abs(-200, 200, 0, True)

    # move forward to get closer to the mission so mission is set up correctly
    await motor_pair.move_for_degrees(motor_pair.PAIR_1, degrees_for_distance(18), 0, velocity=400)

    # move robot back to complete alignment with Artificial Habitat
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.45, ki=0, kd=0, speed=-600, target_angle=0, sleep_time=0, follow_for=follow_for_distance,
        initial_position=initial_position, distance_to_cover=degrees_for_distance(20))

    # bring scooper down to get ready to slightly lift mission up
    await motor.run_for_degrees(port.B, 100, 150)

    # move robot forward to get scooper under artificial habitat
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.45, ki=0, kd=0, speed=300, target_angle=-3, sleep_time=0, follow_for=follow_for_distance,
        initial_position=initial_position, distance_to_cover=degrees_for_distance(9))

    # bring scooper up to complete mission
    await motor.run_for_degrees(port.B, -195, 1050, acceleration=5000)

    # move robot forward to push crab facing up and to align with mission
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.45, ki=0, kd=0, speed=500, target_angle=-8, sleep_time=0, follow_for=follow_for_distance,
        initial_position=initial_position, distance_to_cover=degrees_for_distance(20))

    # move robot back to move away from Artificial Habitat
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.45, ki=0, kd=0, speed=-800, target_angle=0, sleep_time=0, follow_for=follow_for_distance,
        initial_position=initial_position, distance_to_cover=degrees_for_distance(8))

    # turn right to go towards Unexpected encounter dropoff
    await pivot_gyro_turn_abs(300, -300, 57, True)

    # move robot forward to keep going towards Unexpected encounter dropoff
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.45, ki=0, kd=0, speed=1100, target_angle=57, sleep_time=0, follow_for=follow_for_distance,
        initial_position=initial_position, distance_to_cover=degrees_for_distance(35))

# END RUN 5
#----------------------------------------

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
                            1: run1, # or run1a
                            2: run2,
                            3: run3,
                            4: run4,
                            5: run5
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

# SLOT 0 - All Runs#
runloop.run(execute([1, 2, 3, 4, 5]))

# SLOT 1 - Run 2 Onwards
# runloop.run(execute([2, 3, 4, 5]))

# SLOT 2 - Run 3 Onwards
# runloop.run(execute([3, 4, 5]))

# SLOT 3 - Run 4 Onwards
# runloop.run(execute([4, 5]))

# SLOT 4 - Run 5
# runloop.run(execute([5]))
