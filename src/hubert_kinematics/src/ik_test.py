from hubert_kinematics.ik_solver import ForwardKinematics as FK
from hubert_kinematics.ik_solver import InverseKinematics as IK
from math import pi
import random
import numpy as np
import time

solutions_found = 0
iterations = 1
start = time.time()
for i in range(iterations):

    print(f"{i}/{iterations}")

    theta1 = random.uniform(-pi/2, pi/2)
    theta2 = random.uniform(pi/6, pi/2)
    theta3 = random.uniform(-pi/2, 0)

    #print(theta1)

    forward_kinematics = FK(theta1, theta2, theta3)#, pi/2, 0, theta1)
    inverse_kinematics = IK(forward_kinematics.coords[0], forward_kinematics.coords[1], forward_kinematics.coords[2])
    angles = inverse_kinematics.angles

    distances = []
    # print(f"Angles nearest coordinates: {angles}")
    f = FK(angles[0], angles[1], angles[2])
    distances.append(np.linalg.norm(forward_kinematics.coords-f.coords))

    if inverse_kinematics.success == True:
        solutions_found += 1
        

stop = time.time()
print(f"Solutions found: {solutions_found}/{iterations}")
print(f"Time: {stop-start:.3f} s")
print(f"Time per iteration: {(stop-start)/iterations:.3f}")
print(f"Mean distance: {np.mean(np.array(distances))}")