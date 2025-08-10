# EKF SLAM by Subin

## Analyzing SLAM logs

A Python utility script `scripts/analyze_slam_log.py` can be used to inspect a `slam_log.csv` file.
It prints basic statistics for numeric columns and generates trajectory/time-series plots.

### Usage

```bash
python scripts/analyze_slam_log.py path/to/slam_log.csv --out plots
```

The generated plots are saved in the directory specified by `--out` (default: `plots`).

