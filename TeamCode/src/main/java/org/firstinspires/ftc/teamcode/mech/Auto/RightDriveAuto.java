package org.firstinspires.ftc.teamcode.mech.Auto;
import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.mech.control.CustomPIDF;

@Autonomous(name = "Right Drive Auto")

public class RightDriveAuto extends LinearOpMode {
    private static final class Config {
        // 28 as a placeholder for forward distance now.
        static final double FORWARD_INCHES = 28;
        static final Pose2d START_POSE = new Pose2d(12, -60, Math.toRadians(90));

        static final Pose2d SHOOT_POSE = new Pose2d(
                START_POSE.position.x,
                START_POSE.position.y + FORWARD_INCHES,
                START_POSE.heading.toDouble());

        // shooting values from MainTeleop, keep these the same for now.
        static final double TARGET_RPM = 700.0;
        static final double LAUNCHER_TICKS_PER_REV = 28.0;
        static final long FIRST_SPINUP_MS = 2000;
        static final long STABLE_MS = 100;
        static final double RPM_TOL_FRAC = 0.05;
        static final double FEED_POWER = 1.0;
        static final long FEED_MS = 3000;
        static final long RECOVER_MS = 1000;
        static final double NOMINAL_VOLTAGE = 12.0;
        static final double LAUNCH_kP = 0.005;
        static final double LAUNCH_kI = 0.00005;
        static final double LAUNCH_kD = 0.00003;
        static final double LAUNCH_kF = -1.0;
    }

    //subject to change for when there's a clearer plan for auto
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, Config.START_POSE);
        ShootAction shoot = new ShootAction(hardwareMap);
        Action moveForward = drive.actionBuilder(Config.START_POSE)
                .splineToLinearHeading(Config.SHOOT_POSE, Math.toRadians(90))
                .build();

        // return from the endppoint of the forward task
        Action moveBackward = drive.actionBuilder(Config.SHOOT_POSE)
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(Config.START_POSE, Math.toRadians(270))
                .build();

        // each taskd needs to be finished before starting the next
        Action autonomous = new SequentialAction(
                moveForward,
                // finish shooting before moving back.
                shoot,
                moveBackward
        );

        try {
            // wait till driver starts
            waitForStart();
            if (isStopRequested()) return;

            Actions.runBlocking(packet -> {
                if (isStopRequested()) return false;
                return autonomous.run(packet);
            });
        } finally {
            // stop all the drive motors.
            drive.leftFront.setPower(0);
            drive.leftBack.setPower(0);
            drive.rightBack.setPower(0);
            drive.rightFront.setPower(0);
            // stop the launcher too.
            shoot.stop();
        }
    }

    // same shooting cycle as MainTeleop, without the turret aiming thing
    private static final class ShootAction implements Action {
        private enum ShootState { IDLE, START, SPINUP, FIRE, RECOVER }

        private final HardwareMap hardwareMap;
        private final DcMotorEx launcher;
        private final CRServo bottomFlywheel, topFlywheel;
        private final CustomPIDF launcherPIDF;
        private final ElapsedTime shootTimer = new ElapsedTime();
        private final ElapsedTime rpmStableTimer = new ElapsedTime();
        private final ElapsedTime launcherLoopTimer = new ElapsedTime();

        private ShootState shootState = ShootState.START;
        private double launcherTargetTicksPerSec = 0.0;
        private boolean launcherControlEnabled = false;

        ShootAction(HardwareMap hardwareMap) {
            this.hardwareMap = hardwareMap;
            launcher = hardwareMap.get(DcMotorEx.class, "launcher");
            bottomFlywheel = hardwareMap.get(CRServo.class, "bottomFlywheel");
            topFlywheel = hardwareMap.get(CRServo.class, "topFlywheel");

            // set up the launcher the same way as teleop.
            launcher.setDirection(DcMotorEx.Direction.REVERSE);
            launcher.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            launcher.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            launcher.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
            stopShooter();

            double maxRpm = launcher.getMotorType().getMaxRPM();
            double maxTicksPerSec = maxRpm * Config.LAUNCHER_TICKS_PER_REV / 60.0;
            double kF = Config.LAUNCH_kF > 0.0 ? Config.LAUNCH_kF
                    : (maxTicksPerSec > 0.0 ? 1.0 / maxTicksPerSec : 0.0);
            launcherPIDF = new CustomPIDF(Config.LAUNCH_kP, Config.LAUNCH_kI, Config.LAUNCH_kD, kF);
            launcherPIDF.iMax = 0.35;
            launcherPIDF.reset();
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            // keep updating the shot and launcher speed till the cycle finishes.
            updateShooting();
            updateLauncherPIDF();
            return shootState != ShootState.IDLE;
        }

        private void setLauncherRPM(double rpm) {
            launcherTargetTicksPerSec = rpm * Config.LAUNCHER_TICKS_PER_REV / 60.0;
            launcherControlEnabled = rpm > 0.0;
            launcherPIDF.reset();
            launcherLoopTimer.reset();
        }

        private boolean launcherAtSpeed() {
            double rpm = launcher.getVelocity() / Config.LAUNCHER_TICKS_PER_REV * 60.0;
            return Math.abs(rpm - Config.TARGET_RPM) <= Config.RPM_TOL_FRAC * Config.TARGET_RPM;
        }

        private double batteryVoltage() {
            double minV = 99.0;
            for (VoltageSensor sensor : hardwareMap.voltageSensor) {
                double voltage = sensor.getVoltage();
                if (voltage > 0) minV = Math.min(minV, voltage);
            }
            return minV < 99.0 ? minV : Config.NOMINAL_VOLTAGE;
        }

        private void updateLauncherPIDF() {
            if (!launcherControlEnabled) return;

            double dt = launcherLoopTimer.seconds();
            launcherLoopTimer.reset();
            double power = launcherPIDF.update(launcherTargetTicksPerSec, launcher.getVelocity(), dt);
            double scale = Config.NOMINAL_VOLTAGE / batteryVoltage();
            launcher.setPower(Range.clip(power * scale, -1.0, 1.0));
        }

        private void updateShooting() {
            switch (shootState) {
                case IDLE:
                    return;

                case START:
                    setLauncherRPM(Config.TARGET_RPM);
                    rpmStableTimer.reset();
                    shootTimer.reset();
                    shootState = ShootState.SPINUP;
                    break;

                case SPINUP:
                    boolean atSpeed = launcherAtSpeed();
                    if (!atSpeed) rpmStableTimer.reset();

                    boolean stableEnough = atSpeed && rpmStableTimer.milliseconds() >= Config.STABLE_MS;
                    boolean timedOut = shootTimer.milliseconds() >= Config.FIRST_SPINUP_MS;
                    // same as teleop: feed when at speed or when the spinup timer runs out.
                    if (stableEnough || timedOut) {
                        bottomFlywheel.setPower(Config.FEED_POWER);
                        topFlywheel.setPower(Config.FEED_POWER);
                        shootTimer.reset();
                        shootState = ShootState.FIRE;
                    }
                    break;

                case FIRE:
                    if (shootTimer.milliseconds() >= Config.FEED_MS) {
                        bottomFlywheel.setPower(0);
                        topFlywheel.setPower(0);
                        shootTimer.reset();
                        shootState = ShootState.RECOVER;
                    }
                    break;

                case RECOVER:
                    // wait for the rpm to recover or for the recovery timer to run out.
                    if (launcherAtSpeed() || shootTimer.milliseconds() >= Config.RECOVER_MS) {
                        stop();
                    }
                    break;
            }
        }

        private void stopShooter() {
            launcherControlEnabled = false;
            launcherTargetTicksPerSec = 0.0;
            launcher.setPower(0);
            bottomFlywheel.setPower(0);
            topFlywheel.setPower(0);
        }

        void stop() {
            shootState = ShootState.IDLE;
            stopShooter();
        }
    }
}
