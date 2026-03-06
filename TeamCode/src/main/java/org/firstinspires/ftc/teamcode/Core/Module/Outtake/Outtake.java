package org.firstinspires.ftc.teamcode.Core.Module.Outtake;


import static org.firstinspires.ftc.teamcode.Constants.DeviceNames.breakBeamOuttakeName;
import static org.firstinspires.ftc.teamcode.Constants.Globals.BlueGoal;
import static org.firstinspires.ftc.teamcode.Constants.Globals.BlueGoalDistance;
import static org.firstinspires.ftc.teamcode.Constants.Globals.RedGoal;
import static org.firstinspires.ftc.teamcode.Constants.Globals.RedGoalDistance;
import static org.firstinspires.ftc.teamcode.Constants.Globals.autoColor;
import static org.firstinspires.ftc.teamcode.Constants.OuttakeConstants.LinkageCameraConstants.ArtifactPose;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Core.Hardware.HighModule;

public class Outtake extends HighModule {

    public Shooter shooter;
    public Turret turret;
    public LinkageCamera linkageCamera;
    public Blocker blocker;
    public DigitalChannel breakBeamOuttake;
    private final Telemetry telemetry;

    public Pose robotPose = new Pose(0,0,0);
    public final ElapsedTime timer = new ElapsedTime();
    public double distanceToGoal;
    public volatile boolean hasShot = false;
    private Thread breakBeamThread;
    private volatile boolean isRunning = true;
    public boolean isShooting = false;
    public static double gravitationalCoef = 386.1; // in/s^2
    public static double shooterHeight = 14.5;
    public static double kE = 1.25;
    public static double baseClearanceOffset = 5.0;
    public Hood hood;


    public Outtake(HardwareMap hardwareMap, Constants.Color color, Telemetry telemetry , boolean isAuto) {
        shooter = new Shooter(hardwareMap);
        turret = new Turret(hardwareMap, color, telemetry);
        hood = new Hood(hardwareMap , isAuto);
        linkageCamera = new LinkageCamera(hardwareMap,ArtifactPose,isAuto);
        blocker = new Blocker(hardwareMap, Blocker.ClosedPosition, isAuto);
        breakBeamOuttake = hardwareMap.get(DigitalChannel.class, breakBeamOuttakeName);
        breakBeamOuttake.setMode(DigitalChannel.Mode.INPUT);
        this.telemetry = telemetry;
    }

    public void startBreakBeamThread() {
        breakBeamThread = new Thread(() -> {
            boolean lastState = false;
            while (isRunning && isShooting) {
                boolean currentState = breakBeamOuttake.getState();
                if (!hasShot && currentState) {
                    timer.reset();
                    hasShot = true;
                }
                if (hasShot) {
                    if (timer.milliseconds() >= 65) {
                        hasShot = false;
                    }
                }
                lastState = currentState;
            }
        });

        breakBeamThread.setPriority(Thread.MAX_PRIORITY);
        breakBeamThread.start();
    }

    public void stopBreakBeamThread() {
        isRunning = false;
        if (breakBeamThread != null) {
            breakBeamThread.interrupt();
        }
    }

    public void setShootingVelocity(double dist) {
        shooter.setTargetVelocity(dist);
    }

    public void setShootingVelocity() {
        shooter.setTargetVelocity(distanceToGoal);
    }
    public void setRawVelocity(double velocity) {
        shooter.setTargetVelocity(velocity);
    }
    public void setShootingVelocityOffset(double offset) {
        setShootingVelocity(distanceToGoal + offset);
    }
    // E COOKED CEL MAI PROBABIL
    public void setShooterPhysicsForPose(Pose robotPose, GoalPose goal, double trajectoryOffsetDegrees) {
        double distanceToCenter = Math.hypot(goal.x - robotPose.getX(), goal.y - robotPose.getY());
        double distanceToFrontWall = distanceToCenter - goal.frontWallOffsetRadius;
        double frontWallHeightDelta = goal.frontWallZ - shooterHeight;
        double wallClearanceAngleRads = Math.atan2(frontWallHeightDelta, distanceToFrontWall);
        double wallClearanceAngleDegrees = Math.toDegrees(wallClearanceAngleRads);
        double targetAngleDegrees = wallClearanceAngleDegrees + baseClearanceOffset + trajectoryOffsetDegrees;
        targetAngleDegrees = Range.clip(targetAngleDegrees, Constants.OuttakeConstants.HoodConstants.minAngle, Constants.OuttakeConstants.HoodConstants.maxAngle);
        hood.setAngle(targetAngleDegrees);
        double targetHeightDelta = goal.targetZ - shooterHeight;
        double alphaRads = Math.toRadians(targetAngleDegrees);
        double sinA = Math.sin(alphaRads);
        double cosA = Math.cos(alphaRads);
        double denominator = 2 * cosA * (distanceToCenter * sinA - targetHeightDelta * cosA);
        if (denominator > 0) {
            double targetVelocity = 2 * kE * distanceToCenter * Math.sqrt(gravitationalCoef / denominator);
            shooter.setTargetVelocity(targetVelocity);
        } else {
            shooter.setTargetVelocity(0);
        }
    }
    public void setShooterPhysicsForPose(Pose robotPose, GoalPose goal) {
        setShooterPhysicsForPose(robotPose, goal, 0.0);
    }
    public void setShooterPhysics() {
        switch (autoColor){
            case Blue:
                setShooterPhysicsForPose(robotPose, BlueGoal);
                break;
            case Red:
                setShooterPhysicsForPose(robotPose, RedGoal);
                break;
        }
    }
    public void setShooterPhysics(double offsetDeg) {
        switch (autoColor){
            case Blue:
                setShooterPhysicsForPose(robotPose, BlueGoal , offsetDeg);
                break;
            case Red:
                setShooterPhysicsForPose(robotPose, RedGoal , offsetDeg);
                break;
        }
    }


    public void openBlocker() {
        blocker.setState(Blocker.States.Open);
    }

    public void closeBlocker() {
        blocker.setState(Blocker.States.Close);
    }

    public void alignTurret(Pose robotPose) {
        turret.setTarget(turret.getTargetAngleFromDistance(robotPose));
    }
    public void alignTurret(Pose robotPose , double offsetDegrees) {
        turret.setTarget(turret.getTargetAngleFromDistance(robotPose));
        turret.addOffsetDegrees(offsetDegrees);
    }

    public void alignTurret() {
        alignTurret(robotPose);
    }

    //In degrees
    public void offsetTurretToRight(double angle) {
        turret.addOffsetDegrees(-angle);
    }

    public void offsetTurretToLeft(double angle) {
        turret.addOffsetDegrees(angle);
    }

    @Override
    public boolean atTarget() {
        return shooter.atTarget() && turret.atTarget();
    }
    public void increaseToleranceOffset(double offset) {
        shooter.addToleranceOffset(offset);
    }
    public void decreaseToleranceOffset(double offset) {
        shooter.addToleranceOffset(-offset);
    }
    public void setToleranceOffset(double offset) {
        shooter.setToleranceOffset(offset);
    }
    public boolean checkErrorTolerance(double error) {
        return Math.abs(shooter.getVelocityError()) <= error;
    }
    public void addErrorToleranceScaled() {
        double offset;
        if(distanceToGoal <= 180){
            offset = Range.clip(Range.scale(distanceToGoal, 60, 180, 0.5, 0.3), 0.15, 0.5);
        }else{
            offset = 0.15;//TODO maybbe 0.2
        }
        shooter.addToleranceOffset(offset);
    }
    public void addErrorToleranceScaledAuto() {
        double offset;
        if(distanceToGoal <= 180){
            offset = Range.clip(Range.scale(distanceToGoal, 60, 180, 0.4, 0.265), 0.15, 0.5);
        }else{
            offset = 0.15;//TODO maybbe 0.2
        }
        shooter.addToleranceOffset(offset);
    }

    public boolean detectShoot() {
        if (atTarget()) {
            shooter.wasAtTarget = true;
            return false;
        }
        double jerkOffset = 0.35;
        if (shooter.wasAtTarget && (shooter.jerk >= jerkOffset)) {
            shooter.wasAtTarget = false;
            return true;
        }
        return false;
    }

    public void resetErrorTolerance() {
        shooter.setToleranceOffset(0);
    }

    public double getDistanceToGoalTime() {
        return distanceToGoal <= 240 ? 275 : 375;
    }

    @Override
    public void update() {
        shooter.update();
        turret.update();
        blocker.update();
        hood.update();
        linkageCamera.update();
    }

    public void update(Pose robotPose) {
        this.robotPose = robotPose;
        switch (autoColor) {
            case Blue:
                distanceToGoal = 2.54 * Math.hypot(BlueGoalDistance.getX() - robotPose.getX(), BlueGoalDistance.getY() - robotPose.getY());
                break;
            case Red:
                distanceToGoal = 2.54 * Math.hypot(RedGoalDistance.getX() - robotPose.getX(), RedGoalDistance.getY() - robotPose.getY());
                break;
        }
        update();
    }
    public double calculateDistanceToGoal(Pose robotPose) {
        switch (autoColor) {
            case Blue:
                return 2.54 * Math.hypot(BlueGoalDistance.getX() - robotPose.getX(), BlueGoalDistance.getY() - robotPose.getY());
            case Red:
                return 2.54 * Math.hypot(RedGoalDistance.getX() - robotPose.getX(), RedGoalDistance.getY() - robotPose.getY());
        }
        return -1;
    }
    public void setShootingVelocityForPose(Pose pose){
        setShootingVelocity(calculateDistanceToGoal(pose));
    }
    public void setShootingVelocityForPose(Pose pose , double offset){
        setShootingVelocity(calculateDistanceToGoal(pose) + offset);
    }
    public void debug() {
        telemetry.addData("Distance ", distanceToGoal);
        telemetry.addData("Down Velocity", shooter.motorDown.getCurrentVelocity());
        telemetry.addData("Ticks: ", turret.getCurrentTicks());
        telemetry.addData("Target Ticks: ", turret.getTargetTicks());
        telemetry.addData("Power: ", turret.motor.getPower());
        telemetry.addData("Current angle: ", turret.getCurrentAngle());
        telemetry.addData("Current angle degrees: ", turret.getCurrentAngleDegrees());
        telemetry.addData("Target angle: ", turret.getTarget());
        telemetry.addData("Target angle degrees ", turret.getTargetDegrees());
        telemetry.addData("Offset angle : ", turret.getOffset());
    }

}
