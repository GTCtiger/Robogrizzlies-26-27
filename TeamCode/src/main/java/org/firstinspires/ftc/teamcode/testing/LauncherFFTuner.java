package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Measures the launcher feedforward: steps the motor through fixed powers, records the
 * steady-state velocity at each, and fits  power = kS + kF * ticksPerSec.
 * Paste the results into LAUNCH_kS / LAUNCH_kF in MainTeleop.
 *
 * Powers are normalized to 12 V, matching the voltage compensation in MainTeleop.
 * Run with no balls loaded.
 */
@TeleOp(name = "Launcher FF Tuner", group = "tuning")
public class LauncherFFTuner extends LinearOpMode {
    private static final double[] POWERS = {0.1, 0.15, 0.2, 0.3, 0.4, 0.5, 0.6};
    private static final double SETTLE_SEC = 2.0;   // wait for the flywheel to reach steady speed
    private static final double SAMPLE_SEC = 0.75;  // then average velocity over this window
    private static final double MIN_VEL = 20.0;     // ignore points where it didn't break friction
    private static final double NOMINAL_VOLTAGE = 12.0;

    @Override
    public void runOpMode() {
        DcMotorEx launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        // Must match MainTeleop so velocity sign and units are the same
        launcher.setDirection(DcMotorEx.Direction.REVERSE);
        launcher.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        launcher.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        launcher.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        telemetry.addLine("Steps launcher from 10% to 60% power (~20 s). No balls loaded.");
        telemetry.update();
        waitForStart();

        double[] effPower = new double[POWERS.length];
        double[] vel = new double[POWERS.length];
        ElapsedTime t = new ElapsedTime();

        for (int k = 0; k < POWERS.length && opModeIsActive(); k++) {
            launcher.setPower(POWERS[k]);

            t.reset();
            while (opModeIsActive() && t.seconds() < SETTLE_SEC) {
                telemetry.addData("step", "%d/%d  power=%.2f", k + 1, POWERS.length, POWERS[k]);
                telemetry.addData("vel", "%.0f", launcher.getVelocity());
                telemetry.update();
            }

            double velSum = 0, voltSum = 0;
            int n = 0;
            t.reset();
            while (opModeIsActive() && t.seconds() < SAMPLE_SEC) {
                velSum += launcher.getVelocity();
                voltSum += batteryVoltage();
                n++;
            }
            vel[k] = (n > 0) ? velSum / n : 0.0;
            double volts = (n > 0) ? voltSum / n : NOMINAL_VOLTAGE;
            // MainTeleop multiplies controller output by 12/V, so in controller units
            // the power that was actually applied is POWERS[k] * V / 12.
            effPower[k] = POWERS[k] * volts / NOMINAL_VOLTAGE;
        }
        launcher.setPower(0.0);

        // Least-squares fit: effPower = kS + kF * vel  (only points where it actually spun)
        double sx = 0, sy = 0, sxx = 0, sxy = 0;
        int m = 0;
        for (int k = 0; k < POWERS.length; k++) {
            if (vel[k] < MIN_VEL) continue;
            sx += vel[k];
            sy += effPower[k];
            sxx += vel[k] * vel[k];
            sxy += vel[k] * effPower[k];
            m++;
        }
        double denom = m * sxx - sx * sx;
        boolean ok = m >= 2 && Math.abs(denom) > 1e-9;
        double kF = ok ? (m * sxy - sx * sy) / denom : 0.0;
        double kS = ok ? (sy - kF * sx) / m : 0.0;

        while (opModeIsActive()) {
            if (ok) {
                telemetry.addLine("Paste into MainTeleop:");
                telemetry.addData("LAUNCH_kS", "%.4f", kS);
                telemetry.addData("LAUNCH_kF", "%.7f", kF);
            } else {
                telemetry.addLine("Fit failed: launcher didn't spin at 2+ power levels.");
            }
            for (int k = 0; k < POWERS.length; k++) {
                telemetry.addData("p=" + POWERS[k], "vel=%.0f  eff=%.3f", vel[k], effPower[k]);
            }
            telemetry.update();
            idle();
        }
    }

    private double batteryVoltage() {
        double minV = Double.POSITIVE_INFINITY;
        for (VoltageSensor vs : hardwareMap.voltageSensor) {
            double v = vs.getVoltage();
            if (v > 0) minV = Math.min(minV, v);
        }
        return Double.isInfinite(minV) ? NOMINAL_VOLTAGE : minV;
    }
}
