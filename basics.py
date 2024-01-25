from djitellopy import tello  # Import Tello Drone Module

drone = tello.Tello()  # Create Drone Variable

drone.connect()  # Connect to Tello Drone

'''
Before running this, you must be
connected to the Tello drone WIFI.
'''

# Movement Units Measured in cm
drone.move_forward(100)
drone.move_back(100)
drone.move_left(100)
drone.move_right(100)
drone.move_up(100)
drone.move_down(100)

# Turning Units Measured in degrees, out of 3600
drone.rotate_clockwise(1800)
drone.rotate_counter_clockwise(1800)

drone.go_xyz_speed(x=5, y=5, z=5, speed=10)  # x, y, and z relative to drone
drone.curve_xyz_speed(x1=5, y1=5, z1=5, x2=10, y2=10, z2=10, speed=10)
drone.set_speed(x=10)
drone.land()

drone.flip_forward()
drone.flip_back()
drone.flip_right()
drone.flip_left()

drone.get_acceleration_x()
drone.get_speed_x()
drone.get_acceleration_x()
drone.get_highest_temperature()
drone.get_flight_time()
drone.get_height()
drone.get_barometer()
# There are many more commands.
