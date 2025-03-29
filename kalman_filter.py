import numpy as np

class KalmanFilter1D:
    def __init__(self, process_variance=1e-3, measurement_variance=1.0):
        self.x = np.array([[0], [0]])
        self.P = np.eye(2)

        self.F = np.array([[1, 1], [0, 1]])
        self.H = np.array([[1, 0]])

        self.Q = process_variance * np.eye(2)
        self.R = np.array([[measurement_variance]])

    def predict(self):
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q

    def update(self, measurement):
        z = np.array([[measurement]])
        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)

        self.x = self.x + K @ y
        self.P = (np.eye(2) - K @ self.H) @ self.P

    def get_estimate(self):
        return self.x[0, 0]
