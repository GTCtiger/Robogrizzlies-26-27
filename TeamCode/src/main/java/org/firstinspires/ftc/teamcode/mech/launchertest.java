package org.firstinspires.ftc.teamcode.mech;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.mech.control.CustomPIDF;

@TeleOp
public class launchertest extends LinearOpMode {
    private enum ShootState { IDLE, START, SPINUP, FIRE, RECOVER }

    // Keep these launcher settings in sync with MainTeleop.
    private static final double TARGET_RPM = 4600.0;
    private static final double LAUNCHER_TICKS_PER_REV = 28.0;
    private static final double NOMINAL_VOLTAGE = 12.0;
    private static final double LAUNCH_kP = 0.002;
    private static final double LAUNCH_kI = 0.001;
    private static final double LAUNCH_kD = 0.0;
    private static final double LAUNCH_kS = 0.0;
    private static final double LAUNCH_kF = -1.0;
    private static final double LAUNCH_I_ZONE_FRAC = 0.15;
    private static final double RPM_TOL_FRAC = 0.05;
    private static final double FEED_POWER = 1.0;
    private static final long FIRST_SPINUP_MS = 2000;
    private static final long STABLE_MS = 200;
    private static final long FEED_MS = 3000;
    private static final long RECOVER_MS = 1000;

    private DcMotorEx launcher;
    private DcMotorEx backIntake;
    private DcMotorEx middleIntake;
    private CustomPIDF launcherPIDF;
    private ShootState shootState = ShootState.IDLE;
    private double launcherTargetTicksPerSec;
    private double launcherPower;
    private boolean launcherControlEnabled;

    private final ElapsedTime launcherLoopTimer = new ElapsedTime();
    private final ElapsedTime shootTimer = new ElapsedTime();
    private final ElapsedTime rpmStableTimer = new ElapsedTime();

    @Override
    public void runOpMode() {
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        backIntake = hardwareMap.get(DcMotorEx.class, "backIntake");
        middleIntake = hardwareMap.get(DcMotorEx.class, "middleIntake");

        launcher.setDirection(DcMotorEx.Direction.REVERSE);
        launcher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        waitForStart();
        if (isStopRequested()) return;

        double maxTicksPerSec = launcher.getMotorType().getAchieveableMaxTicksPerSecond();
        double kF = LAUNCH_kF > 0.0 ? LAUNCH_kF : (maxTicksPerSec > 0.0 ? 1.0 / maxTicksPerSec : 0.0);
        launcherPIDF = new CustomPIDF(LAUNCH_kP, LAUNCH_kI, LAUNCH_kD, kF);
        launcherPIDF.kS = LAUNCH_kS;
        launcherPIDF.iMax = 0.35;
        launcherPIDF.outputMin = 0.0;
        launcherPIDF.reset();
        launcherLoopTimer.reset();

        // Automatically run one launcher shot cycle when the OpMode starts.
        startShooting();

        while (opModeIsActive()) {
            updateShooting();
            updateLauncherPIDF();

            // Telemetry is limited to launcher performance.
            telemetry.addData("launcherRPM", "%.0f / %.0f", getLauncherRPM(), TARGET_RPM);
            telemetry.addData("launcherPower", "%.2f", launcherPower);
            telemetry.update();
            idle();
        }

        stopShooter();
    }

    private void startShooting() {
        shootState = ShootState.START;
        shootTimer.reset();
    }

    private void updateShooting() {
        switch (shootState) {
            case IDLE:
                return;

            case START:
                setLauncherRPM(TARGET_RPM);
                rpmStableTimer.reset();
                shootTimer.reset();
                shootState = ShootState.SPINUP;
                break;

            case SPINUP: {
                boolean atSpeed = launcherAtSpeed();
                if (!atSpeed) rpmStableTimer.reset();

                boolean stableEnough = atSpeed && rpmStableTimer.milliseconds() >= STABLE_MS;
                boolean timedOut = shootTimer.milliseconds() >= FIRST_SPINUP_MS;
                if (stableEnough || timedOut) {
                    backIntake.setPower(-FEED_POWER);
                    middleIntake.setPower(FEED_POWER);
                    shootTimer.reset();
                    shootState = ShootState.FIRE;
                }
                break;
            }

            case FIRE:
                if (shootTimer.milliseconds() >= FEED_MS) {
                    backIntake.setPower(0.0);
                    middleIntake.setPower(0.0);
                    shootTimer.reset();
                    shootState = ShootState.RECOVER;
                }
                break;

            case RECOVER:
                if (shootTimer.milliseconds() >= RECOVER_MS || launcherAtSpeed()) {
                    shootState = ShootState.IDLE;
                    stopShooter();
                }
                break;
        }
    }

    private void setLauncherRPM(double rpm) {
        launcherTargetTicksPerSec = rpm * LAUNCHER_TICKS_PER_REV / 60.0;
        launcherControlEnabled = rpm > 0.0;
        launcherPIDF.iZone = LAUNCH_I_ZONE_FRAC * launcherTargetTicksPerSec;
        launcherPIDF.reset();
        launcherLoopTimer.reset();
    }

    private double getLauncherRPM() {
        return launcher.getVelocity() / LAUNCHER_TICKS_PER_REV * 60.0;
    }

    private boolean launcherAtSpeed() {
        return Math.abs(getLauncherRPM() - TARGET_RPM) <= RPM_TOL_FRAC * TARGET_RPM;
    }

    private void updateLauncherPIDF() {
        if (!launcherControlEnabled) return;

        double dt = launcherLoopTimer.seconds();
        launcherLoopTimer.reset();
        launcherPower = launcherPIDF.update(launcherTargetTicksPerSec, launcher.getVelocity(), dt);
        launcherPower = Range.clip(launcherPower * NOMINAL_VOLTAGE / batteryVoltage(), 0.0, 1.0);
        launcher.setPower(launcherPower);
    }

    private double batteryVoltage() {
        double minVoltage = 99.0;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0.0) minVoltage = Math.min(minVoltage, voltage);
        }
        return minVoltage < 99.0 ? minVoltage : NOMINAL_VOLTAGE;
    }

    private void stopShooter() {
        launcherControlEnabled = false;
        launcherTargetTicksPerSec = 0.0;
        launcherPower = 0.0;
        launcher.setPower(0.0);
        backIntake.setPower(0.0);
        middleIntake.setPower(0.0);
    }
}
