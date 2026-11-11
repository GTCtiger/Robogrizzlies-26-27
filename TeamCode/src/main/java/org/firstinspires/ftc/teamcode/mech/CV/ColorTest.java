package org.firstinspires.ftc.teamcode.mech.CV;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import org.firstinspires.ftc.teamcode.mech.movement.goBuildaPinPointDriver.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.mech.movement.goBuildaPinPointDriver.Pose2D;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
import java.util.ArrayList;
import java.util.Arrays;

@TeleOp(name="Color Test")
public class ColorTest extends LinearOpMode {
    RevColorSensorV3 color;

    String get_hue(int Red, int Green, int Blue){
        int Max = Math.max(Green, Math.max(Red, Blue));
        int Min = Math.min(Green, Math.min(Red, Blue));
        int delta = Max - Min;
        double hue;
        if (delta==0){
            return "undefined";
        }
        if (Green == Max) {
            hue = 120 + 60.0 * (Blue - Red) / delta;
        }
        else if (Red == Max) {
            hue = 60.0 * (Green - Blue) / delta;
        }
        else {
            hue = 240 + 60.0 * (Red - Green) / delta;
        }
        if (hue < 0) {
            hue += 360;
        }

        if (hue < 30 || hue >= 300) return "Red";

        else if (hue >= 30 && hue < 100) return "Yellow";

        else if (hue >= 180 && hue < 280) return "Blue";

        return "undefined";
    }

    @Override public void runOpMode() {
        color = hardwareMap.get(RevColorSensorV3.class, "colorSensor");
        color.enableLed(true);
        String color_value = "Undef";

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("R", color.red());
            telemetry.addData("G", color.green());
            telemetry.addData("B", color.blue());
            telemetry.addData("A", color.alpha());
            if (color.alpha() > 150) {
                color_value = get_hue(color.red(), color.green(), color.blue());
                telemetry.addData("Color", color_value);
            }
            else {

            }

            telemetry.addData("Color", "idkrn");
            telemetry.update();
        }
    }
}