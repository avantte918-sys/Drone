# PWM7 camera pitch stabilization (AI-assisted local change)

## Backup
Before this change: commit d540145; tag backup/pre-pwm7-mount-20261110.
Create a separate checkout to inspect/restore without discarding work:
`git worktree add ../ardupilot-before-pwm7 backup/pre-pwm7-mount-20261110`
The prior APJ remains separate; restoring source alone does not flash hardware.

## Changes
- MicoAir405v2 hwdef enables HAL_MOUNT_ENABLED and HAL_MOUNT_SERVO_ENABLED.
- Board-only MICOAIR405V2_CAMERA_LEVEL_ON_BOOT enables a startup level target.
- After mount initialization, a Servo primary mount receives roll/pitch/yaw = 0 degrees, body-frame yaw.
- Existing AP_Mount_Servo uses AHRS pitch to compensate and converts angles to PWM.
- Existing NOGPS height, EKF, motor mixer and flight control code are untouched.
- Startup target is not permanent: later mount/ROI commands can change it.

## Required setup after flashing
This firmware deliberately does not overwrite saved output functions or calibration.
1. Remove propellers. Keep servo linkage unloaded during initial setup.
2. Set MNT1_TYPE=1, SERVO7_FUNCTION=7, MNT1_DEFLT_MODE=2; reboot.
3. PWM7/PWM8 share TIM4. Ensure this group uses ordinary servo PWM, not DShot.
4. Use supported PWM refresh frequency (start with 50 Hz if manufacturer supports it).
   SERVO_RATE affects ordinary servo outputs globally, not just channel 7.
5. Calibrate SERVO7_MIN/MAX and MNT1_PITCH_MIN/MAX as matching measured pulse/angle endpoints.
   Both pitch limits must include zero. Start with a small pulse span around 1500 us.
6. Center camera mechanically at the zero-angle command; verify SERVO7_REVERSED.
7. Tilt nose up/down 10 degrees: camera should counter-rotate approximately 10 degrees.
8. Confirm no binding throughout intended aircraft pitch range; check flight-controller supply remains stable.

M005 supply: 3.7-4.2 V only. Use regulated 4.0 V with common ground and PWM7 signal.
Do not connect servo power directly to 5 V or VBAT.
Only pitch is compensated. Servo position is not measured by flight controller.
Camera-to-body geometry now changes; companion vision transforms must account for it.

## Validation scope
Hardware-target compilation and diff checks will be recorded with the delivered artifact.
No physical servo, electrical, scheduler-load or flight validation has been performed.
