"""Preserved steering_tangent filter extracted from its own trio."""
import numpy as np

def cbf_filter(self, v, scan_ranges, scan_angles, u_nominal):
    """
        Parameters
        ----------
        v : float
            Vehicle linear velocity
        scan_ranges : np.ndarray (N,)
            LiDAR ranges
        scan_angles : np.ndarray (N,)
            Corresponding beam angles
        u_nominal : np.ndarray (2,)
            Nominal control [acceleration, angular_speed]

        Returns
        -------
        u_opt : np.ndarray (2,)
            CBF-filtered control
        h : np.ndarray (nBeams,)
            CBF values
        """
    scan_max_range = 10.0
    n_beams = 20
    N = scan_angles.size
    idx = np.linspace(0, N - 1, n_beams)
    psi = np.interp(idx, np.arange(N), scan_angles)
    R = np.interp(idx, np.arange(N), scan_ranges)
    R = np.nan_to_num(R, nan=scan_max_range)
    v_obj = np.zeros(n_beams)
    Td = 0.5 * np.cos(psi) + 0.1
    gamma = 0.7 + 0.3 * np.sin(psi)
    h = np.zeros(n_beams)
    alpha_h = np.zeros(n_beams)
    Lfh = np.zeros(n_beams)
    Lgh = np.zeros((n_beams, 2))
    for i in range(n_beams):
        Lfh[i] = v_obj[i] - v * np.cos(psi[i]) - Td[i] * v ** 2 * np.sin(psi[i]) ** 2 / R[i]
        Lgh[i, :] = [-Td[i] * np.cos(psi[i]), Td[i] * v ** 2 * np.sin(psi[i]) / self.L]
        Dmin = 0.5
        h[i] = R[i] - Td[i] * v * np.cos(psi[i]) - Dmin
        alpha_h[i] = gamma[i] * h[i] ** 3
    u_min = np.array([-3.0, -0.4])
    u_max = np.array([3.0, +0.4])
    A_lim = np.array([[1, 0], [-1, 0], [0, 1], [0, -1]])
    b_lim = np.array([u_max[0], -u_min[0], u_max[1], -u_min[1]])
    H = np.array([[2.0, 0.0], [0.0, 2.0]])
    f = -2.0 * u_nominal
    A_q = -Lgh
    b_q = Lfh + alpha_h
    A = np.vstack((A_q, A_lim))
    b = np.hstack((b_q, b_lim))
    u_opt = self.qp_hildreth(H, f, A, b)
    return (u_opt, h)
