Forecasts up to 3 s ahead, 50 runs pooled

|  | ADE (m) | FDE (m) | FDE straight | FDE turning | error at 1 s | coverage at 1 s | coverage at 3 s | coverage, all |
|---|---|---|---|---|---|---|---|---|
| KF (CV) | 11.24 | 17.80 | 12.15 | 28.01 | 7.49 | 97% | 94% | 96% |
| EKF state, straight ahead | 4.26 | 7.63 | 5.07 | 12.04 | 2.49 | 94% | 77% | 86% |
| EKF, yaw rate held (CTRV) | 3.20 | 4.79 | 5.22 | 4.05 | 2.40 | 95% | 95% | 96% |
| UKF, yaw rate held (CTRV) | 3.15 | 4.70 | 5.12 | 3.99 | 2.37 | 96% | 95% | 96% |
| EKF, yaw rate and accel held | 2.91 | 3.98 | 3.93 | 4.07 | 2.37 | 96% | 96% | 96% |
