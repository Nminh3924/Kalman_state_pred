12748 samples over 127.5 s (100 Hz, longest gap 0.07 s, 6 dropped), reported position accuracy 0.18 m
path 920 m, speed -0.1 to 14.6 m/s, turning faster than 0.05 rad/s 40% of the time
checks:
  ENU path length vs integral of forward speed: 919.9 m vs 919.1 m
  direction of travel minus yaw: mean +0.42 deg, std 1.08 deg
  forward accel minus d(speed)/dt, 1 s means: mean +0.007, std 0.026 m/s^2
  yaw rate minus d(yaw)/dt, 1 s means: mean +0.00008, std 0.00633 rad/s
wrote truth.csv, gps.csv (1 Hz, sigma 3 m, seed 0) and imu.csv to data/kitti/2011_09_30_drive_0034
