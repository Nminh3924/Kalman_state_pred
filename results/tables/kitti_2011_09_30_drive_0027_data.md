11550 samples over 115.5 s (100 Hz, longest gap 0.07 s, 6 dropped), reported position accuracy 0.06 m
path 693 m, speed -0.1 to 11.5 m/s, turning faster than 0.05 rad/s 37% of the time
checks:
  ENU path length vs integral of forward speed: 692.9 m vs 692.3 m
  direction of travel minus yaw: mean -0.14 deg, std 1.10 deg
  forward accel minus d(speed)/dt, 1 s means: mean +0.007, std 0.019 m/s^2
  yaw rate minus d(yaw)/dt, 1 s means: mean +0.00004, std 0.00824 rad/s
wrote truth.csv, gps.csv (1 Hz, sigma 3 m, seed 0) and imu.csv to data/kitti/2011_09_30_drive_0027
