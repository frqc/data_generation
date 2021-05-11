#!/usr/bin/env python

# Copyright (c) 2020 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Sensor synchronization example for CARLA

The communication model for the syncronous mode in CARLA sends the snapshot
of the world and the sensors streams in parallel.
We provide this script as an example of how to syncrononize the sensor
data gathering in the client.
To to this, we create a queue that is being filled by every sensor when the
client receives its data and the main loop is blocked until all the sensors
have received its data.
This suppose that all the sensors gather information at every tick. It this is
not the case, the clients needs to take in account at each frame how many
sensors are going to tick at each frame.
"""

import glob
import os
import sys
from queue import Queue
from queue import Empty
import random
import carla


# Sensor callback.
# This is where you receive the sensor data and
# process it as you liked and the important part is that,
# at the end, it should include an element into the sensor queue.
def sensor_callback(sensor_data, sensor_queue, sensor_name):
    # Do stuff with the sensor_data data like save it to disk
    # Then you just need to add to the queue
    sensor_queue.put((sensor_data, sensor_name))


def get_transform(one_transform, location_id):

    location_delta = one_transform.get_right_vector() * int(location_id) * 0.1

    final_transform = carla.Transform(
        carla.Location(x=one_transform.location.x + location_delta.x,
                       z=one_transform.location.z + location_delta.z,
                       y=one_transform.location.y + location_delta.y))

    return final_transform


def main():
    # We start creating the client
    client = carla.Client('localhost', 2000)
    client.set_timeout(2.0)
    world = client.get_world()

    try:
        # We need to save the settings to be able to recover them at the end
        # of the script to leave the server in the same state that we found it.
        original_settings = world.get_settings()
        settings = world.get_settings()

        # We set CARLA syncronous mode
        settings.fixed_delta_seconds = 0.2
        settings.synchronous_mode = True
        world.apply_settings(settings)
        carla_map = world.get_map()

        # We create the sensor queue in which we keep track of the information
        # already received. This structure is thread safe and can be
        # accessed by all the sensors callback concurrently without problem.
        sensor_queue = Queue()

        # Bluepints for the sensors
        blueprint_library = world.get_blueprint_library()
        cam_bp = blueprint_library.find('sensor.camera.rgb')
        lidar_bp = blueprint_library.find('sensor.lidar.ray_cast')
        radar_bp = blueprint_library.find('sensor.other.radar')

        # We create all the sensors and keep them in a list for convenience.
        sensor_list = []
        # for i in range(5):
        #     cam01 = world.spawn_actor(cam_bp, carla.Transform())
        #     cam01.listen(lambda data: sensor_callback(
        #         data, sensor_queue, "cam_" + str(i)))

        #     sensor_list.append((cam01, i))

        cam00 = world.spawn_actor(cam_bp, carla.Transform())
        cam00.listen(lambda data: sensor_callback(
            data, sensor_queue, "cam_" + str(0)))
        sensor_list.append((cam00, 0))

        cam01 = world.spawn_actor(cam_bp, carla.Transform())
        cam01.listen(lambda data: sensor_callback(
            data, sensor_queue, "cam_" + str(1)))
        sensor_list.append((cam01, 1))

        cam02 = world.spawn_actor(cam_bp, carla.Transform())
        cam02.listen(lambda data: sensor_callback(
            data, sensor_queue, "cam_" + str(2)))
        sensor_list.append((cam02, 2))

        cam03 = world.spawn_actor(cam_bp, carla.Transform())
        cam03.listen(lambda data: sensor_callback(
            data, sensor_queue, "cam_" + str(3)))
        sensor_list.append((cam03, 3))

        cam04 = world.spawn_actor(cam_bp, carla.Transform())
        cam04.listen(lambda data: sensor_callback(
            data, sensor_queue, "cam_" + str(4)))
        sensor_list.append((cam04, 4))

        cam05 = world.spawn_actor(cam_bp, carla.Transform())
        cam05.listen(lambda data: sensor_callback(
            data, sensor_queue, "cam_" + str(5)))
        sensor_list.append((cam05, 5))

        # Initialize locations
        spawn_point = random.choice(carla_map.get_spawn_points())
        for sensor, location_id in sensor_list:
            new_location = get_transform(spawn_point, location_id)
            sensor.set_transform(new_location)

        # Main loop
        while True:
            # Tick the server
            world.tick()
            w_frame = world.get_snapshot().frame

            spawn_point = random.choice(carla_map.get_spawn_points())

            print("\nWorld's frame: %d" % w_frame)

            # Now, we wait to the sensors data to be received.
            # As the queue is blocking, we will wait in the queue.get() methods
            # until all the information is processed and we continue with the next frame.
            # We include a timeout of 1.0 s (in the get method) and if some information is
            # not received in this time we continue.
            try:
                for sensor, location_id in sensor_list:
                    s_frame, sensoer_name = sensor_queue.get(True, 1.0)
                    print("    Frame: %d   Sensor: %s" %
                          (s_frame.frame, sensoer_name))

                    file_name = '_six_out/' + sensoer_name + '/%08d' % w_frame
                    s_frame.save_to_disk(file_name)

                    new_location = get_transform(spawn_point, location_id)
                    sensor.set_transform(new_location)

            except Empty:
                print("    Some of the sensor information is missed")

    finally:
        world.apply_settings(original_settings)
        for sensor in sensor_list:
            sensor.destroy()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print(' - Exited by user.')
