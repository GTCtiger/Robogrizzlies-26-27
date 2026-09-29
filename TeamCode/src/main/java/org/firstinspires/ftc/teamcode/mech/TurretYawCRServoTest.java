package org.firstinspires.ftc.teamcode.mech;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

/**
 * Manual safety test for the yaw actuator.
 *
 * Configure "turretYaw" as a Continuous Rotation Servo before running this OpMode.
 * Hold a D-pad direction only briefly and release it before the turret reaches a
 * mechanical stop.
 */
@TeleOp(name = "Turret Yaw CR Servo Test", group = "Debug")
public class TurretYawCRServoTest extends LinearOpMode {
    private static final String YAW_SERVO_NAME = "turretYaw";
    private static final double TEST_POWER = 0.20;

    @Override
    public void runOpMode() {
        CRServo yawServo = hardwareMap.get(CRServo.class, YAW_SERVO_NAME);

        telemetry.addLine("Turret Yaw CR Servo Test");
        telemetry.addLine("Hold D-pad RIGHT or LEFT to rotate; release to stop.");
        telemetry.addLine("Use short presses and avoid mechanical stops.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double power = 0.0;
            if (gamepad1.dpad_right) {
                power = TEST_POWER;
            } else if (gamepad1.dpad_left) {
                power = -TEST_POWER;
            }

            yawServo.setPower(power);
            telemetry.addData("Yaw power", "%.2f", power);
            telemetry.addData("Result", power == 0.0 ? "Stopped" : "Commanded to rotate");
            telemetry.update();
        }

        yawServo.setPower(0.0);
    }
}
