"""Hildreth kernel shared by three preserved CBF models, without numerical changes."""
import numpy as np

def qp_hildreth(H, f, A, b, max_iter=38, tol=1e-08):
    """
        Solves:
            min 0.5 x^T H x + f^T x
            s.t. A x <= b
        """
    n_constraints = A.shape[0]
    eta = -np.linalg.solve(H, f)
    violations = A @ eta - b
    if np.all(violations <= 0):
        return eta
    H_inv = np.linalg.inv(H)
    P = A @ H_inv @ A.T
    d = A @ H_inv @ f + b
    lambda_ = np.zeros(n_constraints)
    for _ in range(max_iter):
        lambda_prev = lambda_.copy()
        for i in range(n_constraints):
            w = P[i, :] @ lambda_ - P[i, i] * lambda_[i]
            w += d[i]
            if P[i, i] > 1e-12:
                lambda_[i] = max(0.0, -w / P[i, i])
        if np.linalg.norm(lambda_ - lambda_prev) ** 2 < tol:
            break
    eta = -H_inv @ f - H_inv @ A.T @ lambda_
    return eta
