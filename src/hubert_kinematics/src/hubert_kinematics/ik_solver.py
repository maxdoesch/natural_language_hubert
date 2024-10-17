import numpy as np

class ForwardKinematics:
    def __init__(self, theta1, theta2, theta3):
        self.theta1 = theta1
        self.theta2 = theta2
        self.theta3 = theta3
        self.coords = self._compute_forward_kinematics_batch(np.array([theta1]), np.array([theta2]), np.array([theta3]))[0]
        self.quaternion = self._rpy_to_quaternion(np.pi/2, 0, theta1)

    # Rotation and translation matrixes
    @staticmethod
    def _rot_z(theta):
        return np.array([
            [np.cos(theta), -np.sin(theta), 0, 0],
            [np.sin(theta),  np.cos(theta), 0, 0],
            [0,              0,             1, 0],
            [0,              0,             0, 1]
        ])

    @staticmethod
    def _rot_x(theta):
        return np.array([
            [1, 0,              0,             0],
            [0, np.cos(theta), -np.sin(theta), 0],
            [0, np.sin(theta),  np.cos(theta), 0],
            [0, 0,              0,             1]
        ])

    @staticmethod
    def _trans(L_x, L_y, L_z):
        return np.array([
            [1, 0, 0, L_x],
            [0, 1, 0, L_y],
            [0, 0, 1, L_z],
            [0, 0, 0, 1]
        ])
    
    @staticmethod
    def _rpy_to_quaternion(roll, pitch, yaw):
        # Convert roll, pitch, and yaw (in radians) to a quaternion
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)

        # Compute quaternion values
        q_w = cr * cp * cy + sr * sp * sy
        q_x = sr * cp * cy - cr * sp * sy
        q_y = cr * sp * cy + sr * cp * sy
        q_z = cr * cp * sy - sr * sp * cy

        return np.array([q_x, q_y, q_z, q_w])

    @staticmethod
    def _compute_forward_kinematics_batch(theta1_array, theta2_array, theta3_array):
        # Define link lengths
        L1, L2, L3, L4, L5, L6, L7, L8, L9 = 0.055, 0.315, 0.045, 0.108, 0.005, 0.034, 0.015, 0.088, 0.204
        
        end_effector_positions = []

        # Loop through each angle in the batch
        for theta1, theta2, theta3 in zip(theta1_array, theta2_array, theta3_array):
            # Compute transformations
            T01 = ForwardKinematics._rot_z(theta1) @ ForwardKinematics._trans(L6, -L4, L1 + L2 + L3) @ ForwardKinematics._rot_x(np.pi / 2)
            T12 = ForwardKinematics._rot_z(theta2) @ ForwardKinematics._trans(L7, -L8, -L5)
            T23 = ForwardKinematics._rot_z(theta3) @ ForwardKinematics._trans(0, -L9, 0)

            # Full transformation
            T_final = T01 @ T12 @ T23

            # Extract end-effector position
            end_effector = T_final @ np.array([0, 0, 0, 1])
            end_effector_positions.append(end_effector[:3])  # Append only the (x, y, z) coordinates

        return np.array(end_effector_positions)

class InverseKinematics:
    def __init__(self, x: float, y: float, z: float, threshold=0.002, attempts=25):
        """
        InverseKinematics computes the joint angles required to position Hubert's end-effector 
        at a specific target point (x, y, z) in 3D space.
        
        Joint angles are stored in InverseKinematics.angles

        Parameters
        ----------
        x : float
            Target x-coordinate for the end-effector in 3D space.
        y : float
            Target y-coordinate for the end-effector in 3D space.
        z : float
            Target z-coordinate for the end-effector in 3D space.
        threshold : float
            Precision for the end effector and input coordinate
        attempts : int
            Numebr of attempts before stopping calculation
        """

        self.x = x
        self.y = y
        self.z = z
        self.theta_ranges = [(-np.pi/2, np.pi/2), (np.pi/6, np.pi/2), (-np.pi/2, 0)]
        self.resolution1 = 10  # Coarse resolution
        self.resolution2 = 20  # Fine resolution
        self.refine_range = 0.1  # Range for refinement around best angles (best*0.9 - best - best*1.1)
        self.threshold = 0.002
        self.noise_scale = 0.1
        self.attempt_iter = 0 # Stored iterations int
        self.attempts = 25  # Number of attempts to find a solution
        self.success = False

        # Perform inverse kinematics
        self.angles = self._perform_inverse_kinematics()

    def _perform_inverse_kinematics(self):
        best_angles = None
        best_distance = float('inf')

        # Try up to self.attempts times to find a valid solution
        for self.attempt_iter in range(self.attempts):
            # Step 1: Coarse search
            theta1_base, theta2_base, theta3_base = self._generate_noisy_grid(self.resolution1)
            best_theta1, best_theta2, best_theta3, distance = self._search_in_grid(theta1_base, theta2_base, theta3_base)

            if distance > self.threshold*10:
                exit

            # Step 2: Refined search around the best angles
            theta1_refine_range = (max(self.theta_ranges[0][0], best_theta1 - self.refine_range), 
                                   min(self.theta_ranges[0][1], best_theta1 + self.refine_range))
            theta2_refine_range = (max(self.theta_ranges[1][0], best_theta2 - self.refine_range), 
                                   min(self.theta_ranges[1][1], best_theta2 + self.refine_range))
            theta3_refine_range = (max(self.theta_ranges[2][0], best_theta3 - self.refine_range), 
                                   min(self.theta_ranges[2][1], best_theta3 + self.refine_range))

            theta1_base, theta2_base, theta3_base = self._generate_noisy_grid(self.resolution2, theta1_refine_range, theta2_refine_range, theta3_refine_range)
            refined_theta1, refined_theta2, refined_theta3, distance = self._search_in_grid(theta1_base, theta2_base, theta3_base)

            # Check if we found a solution within the threshold
            if distance < self.threshold:
                self.success = True
                #print(f"Solution found! Attempts: {self.attempt_iter+1}/{self.attempts}")
                return np.array([refined_theta1, refined_theta2, refined_theta3])
            
            # Keep track of the best solution found so far
            if distance < best_distance:
                best_distance = distance
                best_angles = np.array([refined_theta1, refined_theta2, refined_theta3])

        # If no solution found within the threshold, return the best result
        print("Maximum attempts reached. Returning the best solution found.")
        return best_angles

    def _generate_noisy_grid(self, resolution, theta1_range=None, theta2_range=None, theta3_range=None):
        """Generate noisy grid of angles within the given ranges."""
        if theta1_range is None: theta1_range = self.theta_ranges[0]
        if theta2_range is None: theta2_range = self.theta_ranges[1]
        if theta3_range is None: theta3_range = self.theta_ranges[2]

        theta1_base = np.linspace(theta1_range[0], theta1_range[1], resolution)
        theta2_base = np.linspace(theta2_range[0], theta2_range[1], resolution)
        theta3_base = np.linspace(theta3_range[0], theta3_range[1], resolution)

        noise1 = np.random.normal(0, self.noise_scale, size=theta1_base.shape)
        noise2 = np.random.normal(0, self.noise_scale, size=theta2_base.shape)
        noise3 = np.random.normal(0, self.noise_scale, size=theta3_base.shape)

        theta1_noisy = theta1_base + noise1
        theta2_noisy = theta2_base + noise2
        theta3_noisy = theta3_base + noise3

        return theta1_noisy, theta2_noisy, theta3_noisy

    def _search_in_grid(self, theta1_base, theta2_base, theta3_base):
        """Search for the best theta angles in the given grid."""
        theta1_grid, theta2_grid, theta3_grid = np.meshgrid(theta1_base, theta2_base, theta3_base)
        theta1_flat = np.ravel(theta1_grid)
        theta2_flat = np.ravel(theta2_grid)
        theta3_flat = np.ravel(theta3_grid)

        # Compute forward kinematics in batch
        coords = ForwardKinematics._compute_forward_kinematics_batch(theta1_flat, theta2_flat, theta3_flat)

        # Calculate distances between each FK result and the target (x, y, z)
        distances = np.linalg.norm(coords - np.array([self.x, self.y, self.z]), axis=1)

        # Find the index with the minimum distance
        min_idx = np.argmin(distances)

        return theta1_flat[min_idx], theta2_flat[min_idx], theta3_flat[min_idx], distances[min_idx]

fk = ForwardKinematics(0, np.pi/3, -np.pi/3)
print(fk.coords)