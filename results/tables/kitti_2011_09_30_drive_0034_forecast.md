Forecasts up to 3 s ahead, 10 runs pooled, kitti_convert.py --oxts data/kitti/2011_09_30_drive_0034/oxts --quiet, filters --accel_sigma 0.4 --gyro_sigma 0.02 --forecast_accel 2 --forecast_yaw 0.2

|  | ADE (m) | FDE (m) | FDE straight | FDE turning | error at 1 s | coverage at 1 s | coverage at 3 s | coverage, all |
|---|---|---|---|---|---|---|---|---|
| KF (CV) | 9.11 | 13.52 | 12.35 | 15.22 | 6.54 | 99% | 100% | 99% |
| EKF state, straight ahead | 3.17 | 5.08 | 4.68 | 5.71 | 2.17 | 95% | 95% | 95% |
| EKF, yaw rate held (CTRV) | 3.03 | 4.86 | 4.74 | 5.06 | 2.12 | 96% | 98% | 97% |
| UKF, yaw rate held (CTRV) | 3.04 | 4.85 | 4.62 | 5.21 | 2.13 | 96% | 99% | 97% |
| EKF, yaw rate and accel held | 2.65 | 4.03 | 3.70 | 4.55 | 2.03 | 97% | 99% | 98% |
