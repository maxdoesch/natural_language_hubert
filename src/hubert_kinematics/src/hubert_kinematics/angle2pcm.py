from math import pi as PI

class Angle2pcm:
    def __init__(self):
        pass

    @staticmethod
    def __calculate(angle, angle_min, angle_max, pcm_min, pcm_max):

        # Linear interpolation from angle to PCM
        pcm_value = pcm_min + (angle - angle_min) * (pcm_max - pcm_min) / (angle_max - angle_min)

        return int(pcm_value)  # Return as an integer

    def body(self, angle: float = 0.0) -> int:
        """
        Function that converts angle for BODY servo to PCM.
        
        :param angle: Angle value [-PI/2, PI/2]
        :return: Corresponding angle in PCM integer.
        """

        angle_min = -PI/2
        angle_max = PI/2

        pcm_min = 2330
        pcm_max = 560

        pcm_value = self.__calculate(angle, angle_min, angle_max, pcm_min, pcm_max)

        return int(pcm_value)  # Return as an integer

    def neck_pan(self, angle: float = 0.0) -> int:
        """
        Function that converts angle for neck_PAN servo to PCM.
        
        :param angle: Angle value [-PI/2, PI/2]
        :return: Corresponding angle in PCM integer.
        """

        angle_min = -PI/2
        angle_max = PI/2

        pcm_min = 550
        pcm_max = 2400

        pcm_value = -self.__calculate(angle, angle_min, angle_max, pcm_min, pcm_max)

        return int(pcm_value)  # Return as an integer

    def neck_tilt(self, angle: float = 0.0) -> int:
        """
        Function that converts angle for neck_TILT servo to PCM.
        
        :param angle: Angle value [-PI/4, PI/2]
        :return: Corresponding angle in PCM integer.
        """

        angle_min = -PI/4
        angle_max = PI/2

        pcm_min = 2300
        pcm_max = 950

        pcm_value = self.__calculate(angle, angle_min, angle_max, pcm_min, pcm_max)

        return int(pcm_value)  # Return as an integer

    def shoulder(self, angle: float = 0.0) -> int:
        """
        Function that converts angle for SHOULDER servo to PCM.
        
        :param angle: Angle value [-PI/6, PI/2]
        :return: Corresponding angle in PCM integer.
        """

        angle_min = 0
        angle_max = PI/2

        pcm_min = 1270
        pcm_max = 2050

        pcm_value = self.__calculate(angle, angle_min, angle_max, pcm_min, pcm_max)

        return int(pcm_value)  # Return as an integer

    def elbow(self, angle: float = 0.0) -> int:
        """
        Function that converts angle for ELBOW servo to PCM.
        
        :param angle: Angle value [-PI/2, PI/2]
        :return: Corresponding angle in PCM integer.
        """

        angle_min = -PI/2*.9
        angle_max = PI/2

        pcm_min = 550
        pcm_max = 2400

        pcm_value = self.__calculate(angle, angle_min, angle_max, pcm_min, pcm_max)

        return int(pcm_value)  # Return as an integer
    
    def gripper(self, dist_value: float = 0.0) -> int:
        """
        Function that converts angle for GRIPPER servo to PCM.
        
        :param angle: Distance value [0 mm, 33 mm]
        :return: Corresponding distance in PCM integer.
        """

        dist_min = 0
        dist_max = 33

        pcm_min = 550
        pcm_max = 2150

        pcm_value = self.__calculate(dist_value, dist_min, dist_max, pcm_max, pcm_min)

        return int(pcm_value)  # Return as an integer