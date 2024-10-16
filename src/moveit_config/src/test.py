import tf.transformations as tf
from math import pi

def rpy_to_quaternion(roll, pitch, yaw):
    # Convert roll, pitch, yaw to a quaternion
    quaternion = tf.quaternion_from_euler(roll, pitch, yaw)
    return quaternion

# Example usage:
roll = pi/2   # No roll
pitch = 0.0  # No pitch
yaw = pi/2   # Example yaw (90 degrees)

quaternion = rpy_to_quaternion(roll, pitch, yaw)

print(f"Quaternion: {quaternion}")
