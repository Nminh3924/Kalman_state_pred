16581 samples over 166.0 s (100 Hz, longest gap 0.07 s, 18 dropped), reported position accuracy 0.11 m
path 1709 m, speed 2.7 to 14.8 m/s, turning faster than 0.05 rad/s 60% of the time
checks:
  ENU path length vs integral of forward speed: 1708.9 m vs 1708.4 m
  direction of travel minus yaw: mean +0.18 deg, std 0.83 deg
  forward accel minus d(speed)/dt, 1 s means: mean +0.013, std 0.023 m/s^2
  yaw rate minus d(yaw)/dt, 1 s means: mean -0.00005, std 0.00581 rad/s
wrote truth.csv, gps.csv (1 Hz, sigma 3 m, seed 0) and imu.csv to data/kitti/2011_09_30_drive_0033
