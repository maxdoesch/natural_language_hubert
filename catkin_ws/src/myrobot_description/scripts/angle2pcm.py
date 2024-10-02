import numpy as np

def __calculate(angle, angle_min, angle_max, pcm_min, pcm_max):
    # Raise error if outside of boundary
    if angle < angle_min or angle > angle_max:
        raise ValueError(f"Angle must be between {angle_min} and {angle_max} radians.")

    # Linear interpolation from angle to PCM
    pcm_value = pcm_min + (angle - angle_min) * (pcm_max - pcm_min) / (angle_max - angle_min)

    return int(pcm_value)  # Return as an integer

def body(angle: float = 0.0) -> int:
    """
    Function that converts angle for BODY servo to PCM.
    
    :param angle: Angle value [-PI/2, PI/2]
    :return: Corresponding angle in PCM integer.
    """

    angle_min = -np.pi/2
    angle_max = np.pi/2

    pcm_min = 560
    pcm_max = 2330

    pcm_value = __calculate(angle, angle_min, angle_max, pcm_min, pcm_max)

    return int(pcm_value)  # Return as an integer

def head_pan(angle: float = 0.0) -> int:
    """
    Function that converts angle for HEAD_PAN servo to PCM.
    
    :param angle: Angle value [-PI/2, PI/2]
    :return: Corresponding angle in PCM integer.
    """

    angle_min = -np.pi/2
    angle_max = np.pi/2

    pcm_min = 550
    pcm_max = 2400

    pcm_value = __calculate(angle, angle_min, angle_max, pcm_min, pcm_max)

    return int(pcm_value)  # Return as an integer

def head_tilt(angle: float = 0.0) -> int:
    """
    Function that converts angle for HEAD_TILT servo to PCM.
    
    :param angle: Angle value [-PI/4, PI/2]
    :return: Corresponding angle in PCM integer.
    """

    angle_min = -np.pi/4
    angle_max = np.pi/2

    pcm_min = 950
    pcm_max = 2400

    pcm_value = __calculate(angle, angle_min, angle_max, pcm_min, pcm_max)

    return int(pcm_value)  # Return as an integer

def shoulder(angle: float = 0.0) -> int:
    """
    Function that converts angle for SHOULDER servo to PCM.
    
    :param angle: Angle value [-PI/6, PI/2]
    :return: Corresponding angle in PCM integer.
    """

    angle_min = -np.pi/6
    angle_max = np.pi/2

    pcm_min = 750
    pcm_max = 2200

    pcm_value = __calculate(angle, angle_min, angle_max, pcm_min, pcm_max)

    return int(pcm_value)  # Return as an integer

def elbow(angle: float = 0.0) -> int:
    """
    Function that converts angle for ELBOW servo to PCM.
    
    :param angle: Angle value [-PI/2, PI/2]
    :return: Corresponding angle in PCM integer.
    """

    angle_min = -np.pi/2
    angle_max = np.pi/2

    pcm_min = 550
    pcm_max = 2400

    pcm_value = __calculate(angle, angle_min, angle_max, pcm_min, pcm_max)

    return int(pcm_value)  # Return as an integer