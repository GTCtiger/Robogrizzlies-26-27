package org.firstinspires.ftc.teamcode.mech;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.mech.control.BasicShooter;
import org.firstinspires.ftc.teamcode.mech.movement.movement;

/** Basic driving and shooting, same hardware as mvp-code */
@TeleOp(name = "MainTeleop")
public class MainTeleop extends LinearOpMode {
    private static double deadzone(double value) {
        return Math.abs(value) < 0.2 ? 0 : value;
    }

    @Override
    public void runOpMode() {
        movement drive = new movement(this, 0, 0, 0);
        BasicShooter shooter = new BasicShooter(hardwareMap);
        DcMotor frontIntake = hardwareMap.get(DcMotor.class, "frontIntake");
        frontIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontIntake.setPower(0);
        drive.move(0, 0, 0);

        telemetry.addLine("Left stick: forward/back and strafe; right stick: rotate");
        telemetry.addLine("Y: shoot; A: cancel shooter/intakes");
        telemetry.addLine("Right bumper: intake; left bumper: reverse intake");
        telemetry.update();

        try {
            waitForStart();
            boolean yPrev = false;
            while (opModeIsActive()) {
                // same stick directions as mvp-code
                drive.move(deadzone(gamepad1.left_stick_x),
                        deadzone(gamepad1.left_stick_y),
                        deadzone(gamepad1.right_stick_x));

                boolean yNow = gamepad1.y;
                boolean shootPressed = yNow && !yPrev;
                yPrev = yNow;

                // hold A to stop shooting and intakes, even if Y or the bumpers are pressed
                if (gamepad1.a) {
                    shooter.stop();
                    frontIntake.setPower(0);
                } else {
                    if (shootPressed) shooter.start();
                    double intakePower = gamepad1.right_bumper ? 1
                            : (gamepad1.left_bumper ? -1 : 0);
                    frontIntake.setPower(shooter.isBusy() ? 0 : intakePower);
                    shooter.setIntakePower(intakePower);
                    shooter.update();
                }

                telemetry.addData("Shooter", shooter.getState());
                telemetry.addData("Launcher RPM", "%.0f / %.0f",
                        shooter.getRpm(), BasicShooter.TARGET_RPM);
                telemetry.update();
                idle();
            }
        } finally {
            drive.move(0, 0, 0);
            frontIntake.setPower(0);
            shooter.stop();
        }
    }
}
