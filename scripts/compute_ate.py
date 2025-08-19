#!/usr/bin/env python3
"""Compute Absolute Trajectory Error between ground truth and SLAM output."""

import math
from typing import List, Tuple

import matplotlib.pyplot as plt
import numpy as np
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
        errors = []
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
        self.get_logger().info(f'ATE RMSE: {rmse:.4f} m')

        plt.figure()
        plt.plot(gt_x, gt_y, label='Ground Truth')
        plt.plot(slam_x, slam_y, label='SLAM')
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
