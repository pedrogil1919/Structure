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
        'm': float(size.attrib['m']),
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
    composition = video.find('composition')
    try:
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

    return image_data, video_data

###############################################################################
# End of file.
###############################################################################
