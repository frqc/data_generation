import numpy as np


def generate_camera_bp(arg, world, blueprint_library):
    """Generates a CARLA blueprint based on the script parameters"""
    camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')
    camera_bp.set_attribute("image_size_x", str(arg.camera_width))
    camera_bp.set_attribute("image_size_y", str(arg.camera_height))

    return camera_bp


def process_camera_data(data, height, width):
    array = np.frombuffer(data, dtype=np.dtype("uint8"))
    array = np.reshape(array, (height, width, 4))
    # array = array[:, :, :3] # ?
    # array = array[:, :, ::-1] # ?

    return array
