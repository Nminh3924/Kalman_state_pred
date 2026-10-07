Forecasts up to 3 s ahead, 10 runs pooled, kitti_convert.py --oxts data/kitti/2011_09_30_drive_0033/oxts --quiet, filters --accel_sigma 0.4 --gyro_sigma 0.02 --forecast_accel 2 --forecast_yaw 0.2

|  | ADE (m) | FDE (m) | FDE straight | FDE turning | error at 1 s | coverage at 1 s | coverage at 3 s | coverage, all |
|---|---|---|---|---|---|---|---|---|
| KF (CV) | 9.78 | 15.00 | 11.44 | 17.38 | 6.78 | 99% | 100% | 99% |
| EKF state, straight ahead | 3.42 | 6.03 | 3.61 | 7.69 | 2.05 | 97% | 95% | 96% |
| EKF, yaw rate held (CTRV) | 2.61 | 4.11 | 3.55 | 4.50 | 1.90 | 98% | 100% | 99% |
| UKF, yaw rate held (CTRV) | 2.56 | 4.03 | 3.41 | 4.45 | 1.88 | 98% | 100% | 99% |
| EKF, yaw rate and accel held | 2.42 | 3.69 | 3.23 | 4.01 | 1.87 | 99% | 100% | 99% |
