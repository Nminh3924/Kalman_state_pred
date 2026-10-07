Position RMSE in metres, 200 runs pooled, simulate.py --gps_outage 30 40

|  | at fixes | all | straight | turning | no GPS | µs/step |
|---|---|---|---|---|---|---|
| GPS raw | 4.24 | – | – | – | – | – |
| KF (CV) | 3.63 | 10.88 | 12.82 | 5.69 | 24.12 | 0.25 |
| EKF (CTRV) | 2.26 | 3.12 | 3.43 | 2.43 | 5.17 | 0.17 |
| UKF (CTRV) | 2.25 | 3.11 | 3.41 | 2.43 | 5.14 | 0.75 |
