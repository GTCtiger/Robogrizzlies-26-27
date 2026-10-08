# Basic controls on mvp-roadrunner

MainTeleop and Right Drive Auto use the drive conventions and motor-fed
launcher setup from origin/mvp-code. These two OpModes do not use a turret,
Limelight, color sorting, or a spindexer. Other existing OpModes retain their
own hardware requirements.

## Hardware configuration

- Drive motors: FL, FR, BL, BR.
- Shooter motor with encoder: launcher.
- Feeder motors: backIntake, middleIntake.
- MainTeleop also uses frontIntake.
- Right Drive Auto also uses the existing Road Runner MecanumDrive and
  PinpointLocalizer, including the pinpoint device and existing calibration.

## MainTeleop

- Left stick up/down: forward/backward.
- Left stick left/right: strafe left/right.
- Right stick left/right: rotate the robot.
- Y press: start one timed shooting cycle. Holding Y does not repeat.
- A hold: cancel shooting and stop all intakes; takes priority over Y/bumpers.
- Right bumper: intake.
- Left bumper: reverse front/back intake; middle intake remains forward,
  matching mvp-code's reverse-intake convention.
- Shooting temporarily owns the feeder motors and stops the front intake.
- Drive remains responsive during shooting. Releasing the sticks stops drive.

## Right Drive Auto

Starting pose is (12, -60), heading 90 degrees:

1. Forward 28 inches.
2. Complete one shooting cycle.
3. Backward 28 inches to the starting position.
4. Strafe left 12 inches.
5. Strafe right 24 inches, crossing the starting position.
6. Strafe left 12 inches back to the starting position.
7. Turn counterclockwise 90 degrees.
8. Turn clockwise 90 degrees to restore the starting heading.

Distances and turn angle are constants at the top of RightDriveAuto.java.
Translation uses constant-heading Road Runner paths; rotation uses turn actions.
Both OpModes stop their motors in a finally block when stopped or completed.

## Shooter settings

BasicShooter.java shares the nonblocking cycle between both OpModes.
It retains mvp-code's 700 RPM target, PIDF gains, voltage compensation,
100 ms stable-speed requirement, 2-second spinup timeout, 3-second feed
window, and up-to-1-second recovery. As in mvp-code, feeding starts when
speed is stable OR spinup times out. A cycle is timed, not a measured ball count.

Physical distances, direction, RPM, and feeding need verification on the robot.
Compilation does not validate the existing drivetrain or odometry calibration.
