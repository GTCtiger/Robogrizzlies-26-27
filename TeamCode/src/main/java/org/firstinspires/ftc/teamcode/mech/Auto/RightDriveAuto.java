package org.firstinspires.ftc.teamcode.mech.Auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.mech.control.BasicShooter;

/** Basic Road Runner movements and stuff */
@Autonomous(name = "Right Drive Auto")
public class RightDriveAuto extends LinearOpMode {
    private static final double FORWARD_INCHES = 5;
    private static final double STRAFE_INCHES = 5;
    private static final double TURN_DEGREES = 90.0;
    private static final Pose2d START_POSE = new Pose2d(12, -60, Math.toRadians(90));

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, START_POSE);
        BasicShooter shooter = new BasicShooter(hardwareMap);
        try {
            // At a 90 degree heading, forward is field +Y and robot-left is field -X.
            Vector2d start = START_POSE.position;
            Vector2d forward = new Vector2d(start.x, start.y + FORWARD_INCHES);
            Vector2d left = new Vector2d(start.x - STRAFE_INCHES, start.y);
            Vector2d right = new Vector2d(start.x + STRAFE_INCHES, start.y);
            Pose2d shootPose = new Pose2d(forward, START_POSE.heading);

            Action moveForward = drive.actionBuilder(START_POSE)
                    .strafeTo(forward)
                    .build();
            Action shoot = new Action() {
                private boolean started;

                @Override
                public boolean run(@androidx.annotation.NonNull com.acmerobotics.dashboard.telemetry.TelemetryPacket packet) {
                    if (!started) {
                        shooter.start();
                        started = true;
                    }
                    shooter.update();
                    packet.put("shooter", shooter.getState());
                    packet.put("launcherRPM", shooter.getRpm());
                    return shooter.isBusy();
                }
            };
            Action remainingMoves = drive.actionBuilder(shootPose)
                    .strafeTo(start)                  // go back to where we started
                    .waitSeconds(0.25)
                    .strafeTo(left)                   // move left
                    .waitSeconds(0.25)
                    .strafeTo(right)                  // move right, past the start
                    .waitSeconds(0.25)
                    .strafeTo(start)                  // move left to get back to the start
                    .turn(Math.toRadians(TURN_DEGREES))   // turn left
                    .waitSeconds(0.25)
                    .turn(Math.toRadians(-TURN_DEGREES))  // turn right to face the same way as before
                    .build();
            Action autonomous = new SequentialAction(moveForward, shoot, remainingMoves);

            telemetry.addLine("Forward 28 in -> shoot -> backward 28 in");
            telemetry.addLine("Left 12 -> right 24 -> left 12 in");
            telemetry.addLine("Turn 90 degrees CCW, then 90 degrees CW");
            telemetry.update();
            waitForStart();
            if (isStopRequested()) return;

            Actions.runBlocking(packet -> !isStopRequested() && autonomous.run(packet));
        } finally {
            drive.leftFront.setPower(0);
            drive.leftBack.setPower(0);
            drive.rightBack.setPower(0);
            drive.rightFront.setPower(0);
            shooter.stop();
        }
    }
}
