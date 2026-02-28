import color, color_sensor, device, motor, motor_pair, orientation, runloop
import hub
import sys

from hub import light_matrix, button, motion_sensor, light, sound, port

WHITE_COLOR_INTENSITY_MIN = 30
BLACK_COLOR_INTENSITY_MAX = 80
COLOR_SENSOR_PORT = port.D

def get_yaw_value():
    return motion_sensor.tilt_angles()[0] * -0.1

def follow_for_color_white():
    return color_sensor.reflection(COLOR_SENSOR_PORT) <= WHITE_COLOR_INTENSITY_MIN

def follow_for_color_black():
    return color_sensor.reflection(COLOR_SENSOR_PORT) >= BLACK_COLOR_INTENSITY_MAX

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
            runloop.sleep_ms(sleep_time)
        # kp value should be +ve for forward movement (positive speed value), and -ve for backward movement (negative speed value)
        motor_pair.move(motor_pair.PAIR_1, int(steering_value), velocity=speed)

    # stop when follow_for condition is met
    motor_pair.stop(motor_pair.PAIR_1, stop=motor.HOLD)


async def test_go_to_white(reverse=False):
    await follow_gyro_angle(kp=-1, ki=-0.0002, kd=-0.2,
                            speed=250*(-1 if reverse else 1), target_angle=0, sleep_time=0, follow_for=follow_for_color_white)

async def test_go_to_black(reverse=False):
    await follow_gyro_angle(kp=-1, ki=-0.0002, kd=-0.2,
                            speed=250*(-1 if reverse else 1), target_angle=0, sleep_time=0, follow_for=follow_for_color_black)


async def mainProgram():
    motor_pair.pair(motor_pair.PAIR_1, port.A, port.E)
    light_matrix.write("0")
    light.color(light.POWER, color.RED)
    motion_sensor.set_yaw_face(motion_sensor.TOP)
    motion_sensor.reset_yaw(0)
    await runloop.sleep_ms(200)
    await test_go_to_black()
    await runloop.sleep_ms(3000)
    await test_go_to_white()
    await runloop.sleep_ms(100)
    await test_go_to_black()

runloop.run(mainProgram())
