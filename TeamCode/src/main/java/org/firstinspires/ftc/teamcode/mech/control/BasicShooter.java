package org.firstinspires.ftc.teamcode.mech.control;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Basic shooting using the same motors as mvp-code.
 * Keep calling update() each loop so shooting keeps going while the bot moves.
 * Shooting controls the feeders till it's done.
 */
public final class BasicShooter {
    public static final double TARGET_RPM = 700.0;
    private static final double TICKS_PER_REV = 28.0;
    private static final double NOMINAL_VOLTAGE = 12.0;
    private static final double RPM_TOLERANCE = 0.05;
    private static final long SPINUP_MS = 2000;
    private static final long STABLE_MS = 100;
    private static final long FEED_MS = 3000;
    private static final long RECOVER_MS = 1000;

    private enum State { IDLE, SPINUP, FIRE, RECOVER }

    private final HardwareMap hardwareMap;
    private final DcMotorEx launcher;
    private final DcMotor backIntake, middleIntake;
    private final CustomPIDF pidf;
    private final ElapsedTime phaseTimer = new ElapsedTime();
    private final ElapsedTime stableTimer = new ElapsedTime();
    private final ElapsedTime loopTimer = new ElapsedTime();
    private State state = State.IDLE;

    public BasicShooter(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        backIntake = hardwareMap.get(DcMotor.class, "backIntake");
        middleIntake = hardwareMap.get(DcMotor.class, "middleIntake");
        launcher.setPower(0);
        launcher.setDirection(DcMotor.Direction.REVERSE);
        launcher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        middleIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double maxTicksPerSec = launcher.getMotorType().getMaxRPM() * TICKS_PER_REV / 60.0;
        double kF = maxTicksPerSec > 0 ? 1.0 / maxTicksPerSec : 0.0;
        pidf = new CustomPIDF(0.005, 0.00005, 0.00003, kF);
        pidf.iMax = 0.35;
        stop();
    }

    public void start() {
        if (isBusy()) return;
        setFeedPower(0, 0);
        pidf.reset();
        phaseTimer.reset();
        stableTimer.reset();
        loopTimer.reset();
        state = State.SPINUP;
    }

    public void update() {
        if (!isBusy()) return;
        boolean atSpeed = Math.abs(getRpm() - TARGET_RPM) <= TARGET_RPM * RPM_TOLERANCE;
        switch (state) {
            case SPINUP:
                if (!atSpeed) stableTimer.reset();
                // feed when it's at speed or the spinup timer runs out, same as mvp-code
                if ((atSpeed && stableTimer.milliseconds() >= STABLE_MS)
                        || phaseTimer.milliseconds() >= SPINUP_MS) {
                    setFeedPower(1, 1);
                    phaseTimer.reset();
                    state = State.FIRE;
                }
                break;
            case FIRE:
                if (phaseTimer.milliseconds() >= FEED_MS) {
                    setFeedPower(0, 0);
                    phaseTimer.reset();
                    state = State.RECOVER;
                }
                break;
            case RECOVER:
                if (atSpeed || phaseTimer.milliseconds() >= RECOVER_MS) stop();
                break;
            default:
                break;
        }
        if (!isBusy()) return;

        double dt = loopTimer.seconds();
        loopTimer.reset();
        double target = TARGET_RPM * TICKS_PER_REV / 60.0;
        double power = pidf.update(target, launcher.getVelocity(), dt);
        launcher.setPower(Range.clip(power * NOMINAL_VOLTAGE / batteryVoltage(), -1, 1));
    }

    /** Intake waits till shooting is done. Power 0 stops both feeders. */
    public void setIntakePower(double power) {
        if (!isBusy()) {
            // middle intake still goes forward when the back intake reverses, same as mvp-code
            setFeedPower(power, Math.abs(power));
        }
    }

    private void setFeedPower(double backPower, double middlePower) {
        backIntake.setPower(backPower);
        middleIntake.setPower(middlePower);
    }

    private double batteryVoltage() {
        double min = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) min = Math.min(min, voltage);
        }
        return Double.isFinite(min) ? min : NOMINAL_VOLTAGE;
    }

    public boolean isBusy() { return state != State.IDLE; }
    public String getState() { return state.name(); }
    public double getRpm() { return launcher.getVelocity() / TICKS_PER_REV * 60.0; }

    public void stop() {
        state = State.IDLE;
        launcher.setPower(0);
        setFeedPower(0, 0);
        pidf.reset();
    }
}
