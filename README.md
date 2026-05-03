# Safe Wheel Tuning Notes

Key bring-up parameters live in `Core/Inc/app_config.h`.

## Motor direction and speed

- `APP_MOTOR_LEFT_SIGN` / `APP_MOTOR_RIGHT_SIGN`: change one of these if a positive target speed makes that wheel spin backward.
- `APP_ENCODER_LEFT_SIGN` / `APP_ENCODER_RIGHT_SIGN`: change one of these if a forward-spinning wheel reports a negative speed.
- `APP_MAX_PWM`: hard PWM limit. TIM3 period is 9999, so 3500 is about 35% duty.
- `APP_PWM_FEEDFORWARD`: minimum kick added after the PID output leaves the deadband. If the car cannot start, raise it gradually, for example 1200, 1500, 1800.
- `APP_GW_BASE_SPEED`: line-following target speed. Start low, then raise it after direction and sensors are correct.

## GW gray sensor tuning

Watch the UART log lines `GW analog:` and `GW norm:`.

- On white floor, `normalized` should be close to 100.
- On black line, `normalized` should be close to 0.
- `line_bits` sets one bit for every channel whose `normalized <= APP_GW_LOW_THRESHOLD`.
- If black and white are both detected as line, lower `APP_GW_LOW_THRESHOLD`.
- If the black line is not detected, raise `APP_GW_LOW_THRESHOLD`.
- If white reads low and black reads high, recheck the measured `APP_GW_CALIBRATION_BLACK` / `APP_GW_CALIBRATION_WHITE` values or the sensor output polarity.

During `APP_MOTOR_ARM_DELAY_MS`, the code keeps both motors at 0 PWM while still printing GW samples.
