# EKF SLAM by Subin

## ATE Evaluation

The package provides a simple script to compare the SLAM trajectory with the
simulation ground truth. The node listens to `/odom` for ground truth and
`/slam_odom` published by `slam_node` and computes the Absolute Trajectory Error
root mean square error (RMSE). The evaluator prints the RMSE and reports the
time step with the maximum error. These metrics are also written to
`ate_metrics.txt`. A smoothed plot comparing both trajectories is saved to
`trajectory_comparison.png`.

Run the evaluator after launching the SLAM node:

```bash
ros2 run ekf_slam compute_ate.py
```
Press `Ctrl+C` to stop recording and output the results.
