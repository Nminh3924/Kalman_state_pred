Position RMSE in metres, 200 runs pooled, simulate.py --outlier_rate 0.05, gate 9.21

|  | at fixes | all | straight | turning | no GPS | µs/step |
|---|---|---|---|---|---|---|
| GPS raw | 7.89 | – | – | – | – | – |
| KF (CV) | 6.55 | 8.62 | 8.27 | 9.22 | – | 0.25 |
| KF + gating | 5.19 | 6.77 | 6.07 | 7.88 | – | 0.26 |
| EKF (CTRV) | 4.60 | 5.10 | 5.02 | 5.25 | – | 0.17 |
| EKF + gating | 2.27 | 2.55 | 2.63 | 2.40 | – | 0.17 |
| UKF (CTRV) | 4.58 | 5.08 | 4.99 | 5.24 | – | 0.75 |
| UKF + gating | 2.26 | 2.54 | 2.61 | 2.39 | – | 0.76 |
