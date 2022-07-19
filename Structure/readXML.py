"""
Created on 28 ene. 2021

@author: pedro.gil@uah.es

Module to read system parameters from a XML file.

"""

# XML support
from xml.etree import ElementTree
from distutils.util import strtobool


def read_structure(xml_file):
    """Read structure dimensions.

    Returns a dictionary with all the dimensions as required by the constructor
    of class Base (see base.py).

    """
    try:
        'Read data structure from XML file'
        element = ElementTree.parse(xml_file)
    except ElementTree.ParseError:
        raise RuntimeError("XML file", xml_file, "is incorrect.")

    # Get units:
    xml_root = element.getroot()
    units = xml_root.attrib['units']
    # Read structure dimensions:
    size = element.find('size')
    structure_size = {
        'a': float(size.attrib['a']),
        'b': float(size.attrib['b']),
        'c': float(size.attrib['c']),
        'd': float(size.attrib['d']),
        'h': float(size.attrib['h']),
        'v': float(size.attrib['v']),
        'g': float(size.attrib['g']),
        'n': float(size.attrib['n'])}
    wheels = element.find('wheels')
    try:
        wheels_radius = {
            'r1': float(wheels.attrib['r1']),
            'r2': float(wheels.attrib['r2']),
            'r3': float(wheels.attrib['r3']),
            'r4': float(wheels.attrib['r4'])}
    except KeyError:
        wheels_radius = {
            'r1': float(wheels.attrib['r']),
            'r2': float(wheels.attrib['r']),
            'r3': float(wheels.attrib['r']),
            'r4': float(wheels.attrib['r'])}

    return units, structure_size, wheels_radius


def read_stairs(xml_file):
    """Read list of stairs.

    Returns a list of dictionaries with all the dimensions as required by the
    constructor of class Stair (see physics.py).

    """
    try:
        'Read data structure from XML file'
        element = ElementTree.parse(xml_file)
    except ElementTree.ParseError:
        raise RuntimeError("XML file " + xml_file + " is incorrect.")

    # Read the list of stairs:
    stairs = element.find('stairs')
    landing = float(stairs.attrib['landing'])
    dimensions = stairs.findall('dimensions')
    stairs_list = []
    for dimension in dimensions:
        stair = {
            'N': int(dimension.attrib['N']),
            'w': float(dimension.attrib['w']),
            'h': float(dimension.attrib['h']),
            'd': float(dimension.attrib['d'])}
        stairs_list.append(stair)

    return stairs_list, landing

#
# def get_dynamics(dynamics, key_value):
#
#     dynamics_data = dynamics.find(key_value)
#     try:
#         data = {
#             'speed': float(dynamics_data.attrib['speed']),
#             'acceleration': float(dynamics_data.attrib(['acceleration'])),
#             'decceleration': float(dynamics_data.attrib(['decceleration']))}
#     except KeyError:
#         data = {'speed': float(dynamics_data.attrib['speed'])}
#     return data


def read_dynamics(xml_file):
    """Read structure dynamics.

    Returns the variables required by the constructor of class Simulator (see
    simulator.py).

    """
    try:
        'Read data structure from XML file'
        element = ElementTree.parse(xml_file)
    except ElementTree.ParseError:
        raise RuntimeError("XML file " + xml_file + " is incorrect.")

    dynamics_data = {}
    dynamics = element.find('dynamics')
    dynamics_data['actuator_up'] = float(dynamics.attrib['actuator_up'])
    dynamics_data['actuator_dw'] = float(dynamics.attrib['actuator_dw'])
    dynamics_data['elevate_up'] = float(dynamics.attrib['elevate_up'])
    dynamics_data['elevate_dw'] = float(dynamics.attrib['elevate_dw'])
    dynamics_data['incline_up'] = float(dynamics.attrib['incline_up'])
    dynamics_data['incline_dw'] = float(dynamics.attrib['incline_dw'])

    dynamics_data['speed'] = float(dynamics.attrib['speed'])
    try:
        dynamics_data['acceleration'] = float(dynamics.attrib['acceleration'])
        dynamics_data['decceleration'] = float(
            dynamics.attrib['decceleration'])
    except KeyError:
        pass

    samples = element.find('samples')
    sample_data = {'sample_time': float(samples.attrib['sample_time']),
                   'time_units': samples.attrib['time_units']}

    return dynamics_data, sample_data


def read_graphics(xml_file):
    """Read graphics parameters.

    Returns the variables required by the constructor of class Graphics (see
    graphics.py).

    """
    try:
        'Read data structure from XML file'
        element = ElementTree.parse(xml_file)
    except ElementTree.ParseError:
        raise RuntimeError("XML file " + xml_file + " is incorrect.")

    graphics = element.find('graphics')
    ###########################################################################
    resolution = graphics.find('resolution')
    if resolution is None:
        raise KeyError("Tag resolution not found in file %s" % xml_file)
    size = (
        int(resolution.attrib['height']),
        int(resolution.attrib['width']))
    image = graphics.find('image')
    try:
        shift = int(image.attrib['shift'])
    except (AttributeError, KeyError):
        shift = 0
    try:
        scale = float(image.attrib['scale'])
    except (AttributeError, KeyError):
        scale = None
    image_data = {
        'size': size,
        'shift': shift,
        'scale': scale}
    ###########################################################################
    rate = graphics.find('framerate')
    try:
        interval = float(rate.attrib['interval'])
    except (AttributeError, KeyError):
        interval = 5.0
    try:
        pause = bool(strtobool(rate.attrib['pause']))
    except (AttributeError, KeyError):
        pause = True
    ###########################################################################
    csv_element = graphics.find('csv_data')
    try:
        csv_data = {'csv_dir': csv_element.attrib['directory']}
        csv_data['actuator1'] = csv_element.attrib['actuator1']
        csv_data['actuator2'] = csv_element.attrib['actuator2']
        csv_data['actuator3'] = csv_element.attrib['actuator3']
        csv_data['actuator4'] = csv_element.attrib['actuator4']
        csv_data['actuator9'] = csv_element.attrib['actuator9']
        csv_data['speed_0'] = csv_element.attrib['speed_0']
        csv_data['speed_1'] = csv_element.attrib['speed_1']
        csv_data['speed_2'] = csv_element.attrib['speed_2']
        csv_data['speed_3'] = csv_element.attrib['speed_3']
    except (AttributeError, KeyError):
        csv_data = {'csv_dir': None}
    # try:
    #     pause = bool(strtobool(rate.attrib['pause']))
    # except (AttributeError, KeyError):
    #     pause = True

    ###########################################################################
    # Set display to True. In case video_dir does not exist, display remains
    # to True, as it is the behaviour described in the xml.
    display = True
    video = graphics.find('video')
    try:
        video_dir = video.attrib['directory']
        display = bool(strtobool(video.attrib['display']))
    except (AttributeError, KeyError):
        video_dir = None

    ###########################################################################
    try:
        composition = video.find('composition')
        comp_dir = composition.attrib['directory']
        try:
            buffer_size = int(composition.attrib['buffer_size'])
        except (AttributeError, KeyError):
            buffer_size = 256
        try:
            units = composition.attrib['units']
        except (AttributeError, KeyError):
            units = ""
        try:
            margin = float(composition.attrib['margin'])
        except (AttributeError, KeyError):
            margin = 5.0
    except (AttributeError, KeyError):
        comp_dir = None
        buffer_size = 0
        units = ""
        margin = 0.0

    video_data = {
        'video_dir': video_dir,
        'buffer_size': buffer_size,
        'display': display,
        'interval': interval,
        'pause': pause,
        'composition': comp_dir,
        'buffer_size': buffer_size,
        'units': units,
        'margin': margin}

    return image_data, video_data, csv_data

###############################################################################
# End of file.
###############################################################################
