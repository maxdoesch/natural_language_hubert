# Define pi as PI
PI = 3.1415926536

print("pcm2angle.py is being executed.")

class Pcm2angle:
    def __init__(self):
        pass

    @staticmethod
    def __calculate(pcm, angle_min, angle_max, pcm_min, pcm_max) -> float:
        # Raise error if PCM is outside of the boundary
        if pcm < pcm_min or pcm > pcm_max:
            raise ValueError(f"PCM must be between {pcm_min} and {pcm_max}.")

        # Linear interpolation from PCM to angle
        angle_value = angle_min + (pcm - pcm_min) * (angle_max - angle_min) / (pcm_max - pcm_min)

        return -angle_value  # Return the angle as a float

    def body(self, pcm: int) -> float:
        """
        Function that converts PCM for BODY servo to angle.
        
        :param pcm: PCM value [560, 2330]
        :return: Corresponding angle in radians.
        """

        angle_min = -PI/2
        angle_max = PI/2

        pcm_min = 560
        pcm_max = 2330

        angle_value = self.__calculate(pcm, angle_min, angle_max, pcm_min, pcm_max)

        return angle_value  # Return as a float

    def head_pan(self, pcm: int) -> float:
        """
        Function that converts PCM for HEAD_PAN servo to angle.
        
        :param pcm: PCM value [550, 2400]
        :return: Corresponding angle in radians.
        """

        angle_min = -PI/2
        angle_max = PI/2

        pcm_min = 550
        pcm_max = 2400

        angle_value = self.__calculate(pcm, angle_min, angle_max, pcm_min, pcm_max)

        return angle_value  # Return as a float

    def head_tilt(self, pcm: int) -> float:
        """
        Function that converts PCM for HEAD_TILT servo to angle.
        
        :param pcm: PCM value [950, 2400]
        :return: Corresponding angle in radians.
        """

        angle_min = -PI/4
        angle_max = PI/2

        pcm_min = 950
        pcm_max = 2400

        angle_value = self.__calculate(pcm, angle_min, angle_max, pcm_min, pcm_max)

        return angle_value  # Return as a float

    def shoulder(self, pcm: int) -> float:
        """
        Function that converts PCM for SHOULDER servo to angle.
        
        :param pcm: PCM value [750, 2200]
        :return: Corresponding angle in radians.
        """

        angle_min = -PI/6
        angle_max = PI/2

        pcm_min = 750
        pcm_max = 2200

        angle_value = self.__calculate(pcm, angle_min, angle_max, pcm_min, pcm_max)

        return -angle_value  # Return as a float

    def elbow(self, pcm: int) -> float:
        """
        Function that converts PCM for ELBOW servo to angle.
        
        :param pcm: PCM value [550, 2400]
        :return: Corresponding angle in radians.
        """

        angle_min = -PI/2
        angle_max = PI/2

        pcm_min = 550
        pcm_max = 2400

        angle_value = self.__calculate(pcm, angle_min, angle_max, pcm_min, pcm_max)

        return -angle_value  # Return as a float
    
    def gripper(self, pcm: int) -> float:
        """
        Function that converts PCM for GRIPPER servo to translation.
        
        :param pcm: PCM value [550, 2150]
        :return: Corresponding distance from 0 - 33 [mm].
        """

        dist_min = 0
        dist_max = 33

        pcm_min = 550
        pcm_max = 2150

        dist_value = self.__calculate(pcm, dist_min, dist_max, pcm_min, pcm_max)

        return dist_value  # Return as a float