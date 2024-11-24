""" 
    General data structures like twists and screws
    @author Wil Louis Rothman
    November 2024 - December 2024
"""

import numpy as np 

class Twist:
    """
        General twist as defined in EECS C106A.

        >>> v, omega = [3, 4, 5], [pi, pi/2, 0]
        >>> xi = Twist(v, omega)
        >>> xi
        Twist(v = [3, 4, 5], ω = [pi, pi/2, 0])
        >>> 2 * xi.as_array()
        np.array([6, 8, 10, 2pi, pi, 0])
        >>> 2 * xi.vector # same as previous
        np.array([6, 8, 10, 2pi, pi, 0])

    """
    def __init__(self, linear_velocity, angular_velocity):
        linear_velocity, angular_velocity = list(linear_velocity), list(angular_velocity)

        assert len(linear_velocity) == 3 and len(angular_velocity) == 3, \
            f"linear and angular velocities must be 3D vectors, \
                instead got {linear_velocity} (length {len(linear_velocity)}) \
                    and {angular_velocity} (length {len(angular_velocity)})"
        
        self.vector = np.array(list(linear_velocity) + list(angular_velocity))

        assert np.issubdtype(self.vector.dtype, np.number), \
            "linear and angular velocities must be representable by a \
                numerical value"
        
    def __repr__(self):
        return f"Twist(v = {self.vector[:3]}, ω = {self.vector[3:]})"

    def as_array(self):
        return self.vector
    

class Revolute(Twist):
    """
        Pure rotation as defined in EECS C106A.
        xi = [-ω x q    ω]^T
        where omega := angular velocity
        and       q := some point on rotation axis
    """
    def __init__(self, omega, q):
        super().__init__(-np.cross(omega, q), omega)

class Prismatic(Twist):
    """
        Pure translation as defined in EECS C106A.
        xi = [v     0]^T
        where v := linear velocity
    """
    def __init__(self, v):
        super().__init__(v, [0, 0, 0])

class GeneralScrew(Twist):
    """
        General screw motion as defined in EECS C106A.
        xi = [-ω x q + hω   ω]^T
        where h := is pitch
    """
    def __init__(self, omega, q, h):
        super().__init__(-np.cross(omega, q) + h * omega, omega)