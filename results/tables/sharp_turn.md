Position RMSE in metres, 200 runs pooled, simulate.py --turn_rate 0.6

|  | at fixes | all | straight | turning | no GPS | µs/step |
|---|---|---|---|---|---|---|
| GPS raw | 4.21 | – | – | – | – | – |
| KF (CV) | 4.23 | 7.69 | 5.24 | 10.88 | – | 0.25 |
| EKF (CTRV) | 2.09 | 2.33 | 2.51 | 1.95 | – | 0.17 |
| UKF (CTRV) | 2.09 | 2.33 | 2.50 | 1.96 | – | 0.75 |
