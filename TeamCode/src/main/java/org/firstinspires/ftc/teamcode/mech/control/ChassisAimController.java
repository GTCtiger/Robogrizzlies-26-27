package org.firstinspires.ftc.teamcode.mech.control;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/** Turns the chassis toward a tag, using odometry to keep aiming after vision drops out. */
public class ChassisAimController {
    public double turnKp = 0.018;
    public double maxTurnPower = 0.5;
    public double minTurnPower = 0.12;
    public double aimTolDeg = 1.5;
    public long settleMs = 120;

    private boolean frozen;
    private boolean hasTarget;
    private double targetFieldX;
    private double targetFieldY;
    private double yawErrorDeg;
    private double turnPower;
    private final ElapsedTime settleTimer = new ElapsedTime();

    public void setFrozen(boolean frozen) {
        this.frozen = frozen;
        turnPower = 0.0;
        settleTimer.reset();
    }

    public boolean isFrozen() {
        return frozen;
    }

    public void updateVisionMeasurement(double robotXIn, double robotYIn, boolean valid, Pose2d pose) {
        if (valid && !frozen) setTargetRobotRelative(robotXIn, robotYIn, pose);
    }

    public void setTargetRobotRelative(double robotXIn, double robotYIn, Pose2d pose) {
        if (frozen) return;
        if (!Double.isFinite(robotXIn) || !Double.isFinite(robotYIn)
                || Math.hypot(robotXIn, robotYIn) < 1.0) return;
        if (!hasTarget) settleTimer.reset();
        double heading = pose.heading.toDouble();
        targetFieldX = pose.position.x + robotXIn * Math.cos(heading) - robotYIn * Math.sin(heading);
        targetFieldY = pose.position.y + robotXIn * Math.sin(heading) + robotYIn * Math.cos(heading);
        hasTarget = true;
    }

    /** Returns the normalized CCW turn command to send to the drive wheels. */
    public double update(Pose2d pose) {
        if (frozen || !hasTarget) {
            yawErrorDeg = 0.0;
            turnPower = 0.0;
            settleTimer.reset();
            return turnPower;
        }

        double dx = targetFieldX - pose.position.x;
        double dy = targetFieldY - pose.position.y;
        yawErrorDeg = Math.toDegrees(Math.atan2(dy, dx) - pose.heading.toDouble());
        yawErrorDeg = (yawErrorDeg + 180.0) % 360.0;
        if (yawErrorDeg < 0.0) yawErrorDeg += 360.0;
        yawErrorDeg -= 180.0;

        if (Math.abs(yawErrorDeg) <= aimTolDeg) {
            turnPower = 0.0;
        } else {
            settleTimer.reset();
            turnPower = Range.clip(turnKp * yawErrorDeg, -maxTurnPower, maxTurnPower);
            if (Math.abs(turnPower) < minTurnPower) {
                turnPower = Math.signum(yawErrorDeg) * minTurnPower;
            }
        }
        return turnPower;
    }

    public double getYawErrorDeg() { return yawErrorDeg; }
    public double getTurnPower() { return turnPower; }
    public boolean hasTarget() { return hasTarget; }
    public boolean isAimed() {
        return hasTarget && !frozen && Math.abs(yawErrorDeg) <= aimTolDeg
                && settleTimer.milliseconds() >= settleMs;
    }
}
