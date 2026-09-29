"""Planar EKF for OPS/HWT pose and encoder-derived body velocity."""

from dataclasses import dataclass
import math

import numpy as np


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass(frozen=True)
class WheelOdometryEkfConfig:
    ops_position_std_m: float = 0.02
    hwt_yaw_std_rad: float = 0.015
    wheel_forward_std_m_s: float = 0.08
    wheel_lateral_std_m_s: float = 0.16
    wheel_yaw_std_rad_s: float = 0.12
    linear_accel_std_m_s2: float = 1.5
    yaw_accel_std_rad_s2: float = 1.5
    max_dt_s: float = 0.20


class PlanarWheelOdometryEkf:
    """State is x, y, yaw, body-vx, body-vy and yaw-rate."""

    def __init__(self, config: WheelOdometryEkfConfig | None = None) -> None:
        self.config = config or WheelOdometryEkfConfig()
        self.state = np.zeros(6, dtype=np.float64)
        self.covariance = np.eye(6, dtype=np.float64)
        self.initialized = False
        self.last_stamp_cs: int | None = None
        self.wheel_valid = False

    def reset(self) -> None:
        self.state.fill(0.0)
        self.covariance = np.eye(6, dtype=np.float64)
        self.initialized = False
        self.last_stamp_cs = None
        self.wheel_valid = False

    @property
    def pose(self) -> tuple[float, float, float]:
        return tuple(float(value) for value in self.state[:3])

    @property
    def twist(self) -> tuple[float, float, float]:
        return tuple(float(value) for value in self.state[3:])

    def update_pose(self, x_m: float, y_m: float, yaw_rad: float) -> bool:
        yaw_rad = normalize_angle(yaw_rad)
        if not self.initialized:
            velocity = self.state[3:].copy()
            self.state[:] = (x_m, y_m, yaw_rad, *velocity)
            self.covariance = np.diag(
                [
                    self.config.ops_position_std_m**2,
                    self.config.ops_position_std_m**2,
                    self.config.hwt_yaw_std_rad**2,
                    self.config.wheel_forward_std_m_s**2,
                    self.config.wheel_lateral_std_m_s**2,
                    self.config.wheel_yaw_std_rad_s**2,
                ]
            )
            self.initialized = True
            return True

        measurement = np.array((x_m, y_m, yaw_rad), dtype=np.float64)
        observation = np.zeros((3, 6), dtype=np.float64)
        observation[0, 0] = 1.0
        observation[1, 1] = 1.0
        observation[2, 2] = 1.0
        noise = np.diag(
            [
                self.config.ops_position_std_m**2,
                self.config.ops_position_std_m**2,
                self.config.hwt_yaw_std_rad**2,
            ]
        )
        innovation = measurement - observation @ self.state
        innovation[2] = normalize_angle(float(innovation[2]))
        self._correct(observation, noise, innovation)
        self.state[2] = normalize_angle(float(self.state[2]))
        return True

    def update_wheel(
        self,
        forward_m_s: float,
        left_m_s: float,
        yaw_ccw_rad_s: float,
        online_mask: int,
        stamp_cs: int,
    ) -> bool:
        stamp_cs &= 0xFFFF
        previous_stamp = self.last_stamp_cs
        self.last_stamp_cs = stamp_cs
        self.wheel_valid = (online_mask & 0x0F) == 0x0F
        if not self.wheel_valid:
            return False

        measurement = np.array(
            (forward_m_s, left_m_s, yaw_ccw_rad_s), dtype=np.float64
        )
        if not self.initialized:
            self.state[3:] = measurement
            return True

        if previous_stamp is not None:
            delta_cs = (stamp_cs - previous_stamp) & 0xFFFF
            dt = delta_cs * 0.01
            if 0.0 < dt <= self.config.max_dt_s:
                self._predict(dt)

        observation = np.zeros((3, 6), dtype=np.float64)
        observation[0, 3] = 1.0
        observation[1, 4] = 1.0
        observation[2, 5] = 1.0
        noise = np.diag(
            [
                self.config.wheel_forward_std_m_s**2,
                self.config.wheel_lateral_std_m_s**2,
                self.config.wheel_yaw_std_rad_s**2,
            ]
        )
        innovation = measurement - observation @ self.state
        self._correct(observation, noise, innovation)
        return True

    def _predict(self, dt: float) -> None:
        x_m, y_m, yaw_rad, forward_m_s, left_m_s, yaw_rate = self.state
        cosine = math.cos(yaw_rad)
        sine = math.sin(yaw_rad)
        world_vx = cosine * forward_m_s - sine * left_m_s
        world_vy = sine * forward_m_s + cosine * left_m_s
        self.state[0] = x_m + world_vx * dt
        self.state[1] = y_m + world_vy * dt
        self.state[2] = normalize_angle(yaw_rad + yaw_rate * dt)

        transition = np.eye(6, dtype=np.float64)
        transition[0, 2] = (-sine * forward_m_s - cosine * left_m_s) * dt
        transition[0, 3] = cosine * dt
        transition[0, 4] = -sine * dt
        transition[1, 2] = (cosine * forward_m_s - sine * left_m_s) * dt
        transition[1, 3] = sine * dt
        transition[1, 4] = cosine * dt
        transition[2, 5] = dt

        linear_position_std = 0.5 * self.config.linear_accel_std_m_s2 * dt * dt
        yaw_position_std = 0.5 * self.config.yaw_accel_std_rad_s2 * dt * dt
        process_noise = np.diag(
            [
                linear_position_std**2,
                linear_position_std**2,
                yaw_position_std**2,
                (self.config.linear_accel_std_m_s2 * dt) ** 2,
                (self.config.linear_accel_std_m_s2 * dt) ** 2,
                (self.config.yaw_accel_std_rad_s2 * dt) ** 2,
            ]
        )
        self.covariance = (
            transition @ self.covariance @ transition.T + process_noise
        )

    def _correct(
        self,
        observation: np.ndarray,
        noise: np.ndarray,
        innovation: np.ndarray,
    ) -> None:
        innovation_covariance = (
            observation @ self.covariance @ observation.T + noise
        )
        gain = np.linalg.solve(
            innovation_covariance.T,
            (self.covariance @ observation.T).T,
        ).T
        self.state += gain @ innovation
        identity = np.eye(6, dtype=np.float64)
        residual = identity - gain @ observation
        self.covariance = (
            residual @ self.covariance @ residual.T + gain @ noise @ gain.T
        )
        self.covariance = 0.5 * (self.covariance + self.covariance.T)
