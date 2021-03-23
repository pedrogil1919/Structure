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

    # Read structure dimensions:
    size = element.find('size')
    structure_size = {
        'a': float(size.attrib['a']),
        'b': float(size.attrib['b']),
        'c': float(size.attrib['c']),
        'd': float(size.attrib['d']),
        'g': float(size.attrib['g'])}
    wheels = element.find('wheels')
    wheels_radius = {
        'r1': float(wheels.attrib['r1']),
        'r2': float(wheels.attrib['r2']),
        'r3': float(wheels.attrib['r3']),
        'r4': float(wheels.attrib['r4'])}
    return structure_size, wheels_radius


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


def read_simulator(xml_file):
    """Read simulator speeds.

    Returns the variables required by the constructor of class Simulator (see
    simulator.py).

    """
    try:
        'Read data structure from XML file'
        element = ElementTree.parse(xml_file)
    except ElementTree.ParseError:
        raise RuntimeError("XML file " + xml_file + " is incorrect.")

    speed = element.find('speed')
    speed_data = {
        'sample_time': float(speed.attrib['sample_time']),
        'wheel': float(speed.attrib['wheel']),
        'actuator_up': float(speed.attrib['actuator_up']),
        'actuator_dw': float(speed.attrib['actuator_dw']),
        'incline_up': float(speed.attrib['incline_up']),
        'incline_dw': float(speed.attrib['incline_dw']),
        'elevate_up': float(speed.attrib['elevate_up']),
        'elevate_dw': float(speed.attrib['elevate_dw'])
        }
    return speed_data


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
    try:
        resolution = graphics.find('resolution')
        size = (
            int(resolution.attrib['height']),
            int(resolution.attrib['width']))
    except (AttributeError, KeyError):
        size = (720, 1280)
    image = graphics.find('image')
    try:
        shift = int(image.attrib['shift'])
    except (AttributeError, KeyError):
        shift = size[0]/2
    try:
        scale = float(image.attrib['scale'])
    except (AttributeError, KeyError):
        scale = None
    image_data = {
        'size': size,
        'shift': shift,
        'scale': scale}
    ###########################################################################
    display = True
    try:
        video = graphics.find('video')
        video_dir = video.attrib['directory']
        display = bool(strtobool(video.attrib['display']))
    except (AttributeError, KeyError):
        video_dir = None
    try:
        rate = graphics.find('framerate')
        interval = float(rate.attrib['interval'])
        pause = bool(strtobool(rate.attrib['pause']))
    except (AttributeError, KeyError):
        interval = 5.0

    video_data = {
        'video_dir': video_dir,
        'display': display,
        'interval': interval,
        'pause': pause}

    return image_data, video_data

###############################################################################
# End of file.
###############################################################################
