#!/usr/bin/env python3
"""Compute Absolute Trajectory Error between ground truth and SLAM output."""

import math
from typing import List, Tuple

import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import make_interp_spline
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


class ATEEvaluator(Node):
    """Node that records two trajectories and computes ATE RMSE."""

    def __init__(self) -> None:
        super().__init__('ate_evaluator')
        self.gt_traj: List[Tuple[float, Tuple[float, float]]] = []
        self.slam_traj: List[Tuple[float, Tuple[float, float]]] = []
        self.sync_threshold = self.declare_parameter('sync_threshold', 0.05).value

        self.create_subscription(Odometry, '/odom', self._gt_callback, 10)
        self.create_subscription(Odometry, '/slam_odom', self._slam_callback, 10)
        self.get_logger().info('ATE evaluator initialized.')

    def _gt_callback(self, msg: Odometry) -> None:
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.gt_traj.append((t, p))

    def _slam_callback(self, msg: Odometry) -> None:
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.slam_traj.append((t, p))

    def compute_ate(self) -> None:
        """Synchronize trajectories and compute RMSE."""
        if not self.gt_traj or not self.slam_traj:
            self.get_logger().warn('Insufficient data to compute ATE.')
            return

        self.gt_traj.sort(key=lambda x: x[0])
        self.slam_traj.sort(key=lambda x: x[0])
        i = j = 0
        errors: List[float] = []
        pair_times: List[float] = []
        gt_x: List[float] = []
        gt_y: List[float] = []
        slam_x: List[float] = []
        slam_y: List[float] = []

        while i < len(self.gt_traj) and j < len(self.slam_traj):
            t_gt, p_gt = self.gt_traj[i]
            t_slam, p_slam = self.slam_traj[j]
            dt = abs(t_gt - t_slam)
            if dt < self.sync_threshold:
                diff_x = p_gt[0] - p_slam[0]
                diff_y = p_gt[1] - p_slam[1]
                errors.append(math.hypot(diff_x, diff_y))
                pair_times.append((t_gt + t_slam) / 2.0)
                gt_x.append(p_gt[0])
                gt_y.append(p_gt[1])
                slam_x.append(p_slam[0])
                slam_y.append(p_slam[1])
                i += 1
                j += 1
            elif t_gt < t_slam:
                i += 1
            else:
                j += 1

        if not errors:
            self.get_logger().warn('No synchronized pairs found.')
            return

        rmse = math.sqrt(np.mean(np.square(errors)))
        max_error = float(np.max(errors))
        max_idx = int(np.argmax(errors))
        max_time = pair_times[max_idx]
        self.get_logger().info(f'ATE RMSE: {rmse:.4f} m')
        self.get_logger().info(
            f'Max error {max_error:.4f} m at t={max_time:.2f}s '
            f'(GT=({gt_x[max_idx]:.2f}, {gt_y[max_idx]:.2f}), '
            f'SLAM=({slam_x[max_idx]:.2f}, {slam_y[max_idx]:.2f}))'
        )
        print(f'ATE RMSE: {rmse:.4f} m')
        print(
            f'Max error {max_error:.4f} m at t={max_time:.2f}s '
            f'(GT=({gt_x[max_idx]:.2f}, {gt_y[max_idx]:.2f}), '
            f'SLAM=({slam_x[max_idx]:.2f}, {slam_y[max_idx]:.2f}))'
        )
        with open('ate_metrics.txt', 'w', encoding='utf-8') as f:
            f.write(f'RMSE: {rmse:.4f} m\n')
            f.write(
                f'Max error: {max_error:.4f} m at t={max_time:.2f}s\n'
            )

        plt.figure()
        times = np.array(pair_times)
        gt_x_arr = np.array(gt_x)
        gt_y_arr = np.array(gt_y)
        slam_x_arr = np.array(slam_x)
        slam_y_arr = np.array(slam_y)

        if len(times) >= 4:
            t_new = np.linspace(times[0], times[-1], len(times) * 10)
            gt_x_smooth = make_interp_spline(times, gt_x_arr)(t_new)
            gt_y_smooth = make_interp_spline(times, gt_y_arr)(t_new)
            slam_x_smooth = make_interp_spline(times, slam_x_arr)(t_new)
            slam_y_smooth = make_interp_spline(times, slam_y_arr)(t_new)
        else:
            gt_x_smooth, gt_y_smooth = gt_x_arr, gt_y_arr
            slam_x_smooth, slam_y_smooth = slam_x_arr, slam_y_arr

        plt.plot(gt_x_smooth, gt_y_smooth, label='Ground Truth')
        plt.plot(slam_x_smooth, slam_y_smooth, label='SLAM')
        plt.xlabel('X [m]')
        plt.ylabel('Y [m]')
        plt.title(f'Trajectory Comparison (RMSE={rmse:.4f} m)')
        plt.legend()
        plt.axis('equal')
        plt.grid(True)
        plt.savefig('trajectory_comparison.png')
        plt.show()


def main() -> None:
    rclpy.init()
    node = ATEEvaluator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.compute_ate()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
