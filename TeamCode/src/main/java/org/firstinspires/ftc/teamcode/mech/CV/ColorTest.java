package org.firstinspires.ftc.teamcode.mech.CV;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
@TeleOp(name = "Color and Distance Test")
public class ColorTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        RevColorSensorV3 colorSensor = hardwareMap.get(RevColorSensorV3.class, "colorSensor");

        DistanceSensor distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor1");

        colorSensor.enableLed(true);
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("R", colorSensor.red());
            telemetry.addData("G", colorSensor.green());
            telemetry.addData("B", colorSensor.blue());

            telemetry.addData("Distance 1 (cm)", distanceSensor.getDistance(DistanceUnit.CM));
            telemetry.update();
        }
    }
}
//link up both sensors, one for 2 color sensors, one for a color sensor and 2 distance to check rad of ball.