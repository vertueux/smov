Remember to align your servos before mounting. Also to test them before you mount, in case some are presenting excessive jitter. To thest them use the script  `servo_calibration.py` in this folder. Make sure you have the I2C interface connected in raspberry, in the **Interfacing Options**.
```
sudo raspi-config
```

```
ros2 run i2c_servos calibration
```