import numpy as np
import cmath

def ackermann_pole_placement(A, B, desired_poles):
    """
    Calculate the state feedback gain matrix K using the Ackermann method
    for pole placement.

    Parameters:
    A (numpy.ndarray): The state matrix of the system.
    B (numpy.ndarray): The input matrix of the system.
    desired_poles (list or numpy.ndarray): The desired eigenvalues (poles) for the closed-loop system.

    Returns:
    numpy.ndarray: The gain matrix K for state feedback.
    """
    # Check if the system is controllable
    n = A.shape[0]
    controllability_matrix = np.hstack([np.linalg.matrix_power(A, i) @ B for i in range(n)])
    if np.linalg.matrix_rank(controllability_matrix) < n:
        raise ValueError("The system is not controllable and pole placement is not possible.")

    # Calculate the desired characteristic polynomial
    desired_char_poly = np.poly(desired_poles)
    desired_char_poly_coeffs = desired_char_poly[::-1]  # Reverse order for desired polynomial coefficients

    # Calculate the characteristic polynomial of A
    A_char_poly = np.poly(A)
    A_char_poly_coeffs = A_char_poly[::-1]  # Reverse order for A polynomial coefficients

    # Compute Ackermann formula for K
    multiply_mat = np.zeros((1, n-1))
    multiply_mat = np.append(multiply_mat, 1).reshape(1, n)
    
    # Calculate the difference between system and desired characteristic polynomials
    phi_A = np.zeros_like(A, dtype=float)
    for i, coeff in enumerate(desired_char_poly_coeffs):
        phi_A += coeff * np.linalg.matrix_power(A, i)

    # Calculate the feedback gain matrix K
    K = multiply_mat @ np.linalg.inv(controllability_matrix) @ phi_A 
    return K

