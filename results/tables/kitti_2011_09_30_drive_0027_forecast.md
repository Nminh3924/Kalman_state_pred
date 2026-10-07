Forecasts up to 3 s ahead, 10 runs pooled, kitti_convert.py --oxts data/kitti/2011_09_30_drive_0027/oxts --quiet, filters --accel_sigma 0.4 --gyro_sigma 0.02 --forecast_accel 2 --forecast_yaw 0.2

|  | ADE (m) | FDE (m) | FDE straight | FDE turning | error at 1 s | coverage at 1 s | coverage at 3 s | coverage, all |
|---|---|---|---|---|---|---|---|---|
| KF (CV) | 9.28 | 13.98 | 11.85 | 17.63 | 6.57 | 99% | 100% | 99% |
| EKF state, straight ahead | 3.16 | 5.38 | 4.15 | 7.69 | 1.99 | 97% | 89% | 94% |
| EKF, yaw rate held (CTRV) | 2.93 | 4.92 | 4.16 | 6.34 | 1.93 | 98% | 94% | 97% |
| UKF, yaw rate held (CTRV) | 2.89 | 4.78 | 3.92 | 6.39 | 1.92 | 98% | 94% | 97% |
| EKF, yaw rate and accel held | 2.47 | 4.01 | 3.39 | 5.19 | 1.81 | 98% | 98% | 99% |
