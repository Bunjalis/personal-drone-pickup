import numpy as np
import casadi as cs



if __name__ == "__main__":
        mass = 1.0

        J = np.array([0.001799424313, 0.001522934832, 0.002923509135])
        thrust_constant = 1.42e-06
        moment_constant = 0.2
        mot_pos_vec = np.array([[0.1, 0.1, 0.1],
                                    [-0.1, 0.1, 0.1], 
                                    [0.1, -0.1, 0.1],
                                    [-0.1, -0.1, 0.1],
                                    [0.1, 0.1, -0.1],
                                    [-0.1, 0.1, -0.1],
                                    [0.1, -0.1, -0.1],
                                    [-0.1, -0.1, -0.1]])

        mot_rot_vec = np.array([[-0.788675,  0.211325,   0.57735],     # CW
                                    [0.211325, 0.788675, -0.57735],         # CW
                                    [-0.211325, -0.788675,  -0.57735],      # CW
                                    [0.788675, -0.211325,   0.57735],       # CW
                                    [0.788675, -0.211325, 0.57735],         # CCW
                                    [-0.211325, -0.788675,  -0.57735],      # CCW
                                    [0.211325, 0.788675, -0.57735],         # CCW
                                    [-0.788675,  0.211325,   0.57735]])     # CCW

        curr = [0., 0., 0.98, 1., 0.02,  0.00, -0., 0.01,   0.0, 0.01, 0.67, -0.57, -0.]
        meas = [0., 0., 0.98, 1., 0.02, -0.02, -0., 0.01, -0.01, 0.01, 0.17, -1.04, -0.]
        pred = [0., 0., 0.98, 1., 0.02, -0.01, -0., 0.01,   0.0, 0.03, 0.39, -0.35, -0.]


        u_action = np.array([0.28, -0.27, -0.28, 0.27, 0.27, -0.28, -0.27, 0.28])


        print("Omnicopter dynamics initialized with mass:", mass)



        thrusts = thrust_constant * 4631 ** 2 * u_action

        torque = np.zeros(3)
        for i in range(8):
                torque += thrusts[i] * cs.cross(mot_rot_vec[i], mot_pos_vec[i])
                print("Motor", i, "torque:", thrusts[i] * cs.cross(mot_pos_vec[i], mot_rot_vec[i]))

        print("Torque:", torque)

                # Compute angular acceleration
        J_inv = cs.diag(1 / J)  # Inverse of inertia matrix
        angular_acceleration = cs.mtimes(J_inv, torque - cs.cross(curr[10:13], cs.mtimes(cs.diag(J), curr[10:13])))

        print("Angular acceleration:", angular_acceleration)
        print("angular velocity contribution:", angular_acceleration *  1/30.0)
        print("angular velocity total:", angular_acceleration * 1/30.0 + curr[10:13])


