package org.firstinspires.ftc.teamcode.Core.Module.Outtake;


import static org.firstinspires.ftc.teamcode.Constants.DeviceNames.breakBeamOuttakeName;
import static org.firstinspires.ftc.teamcode.Constants.Globals.BlueGoal;
import static org.firstinspires.ftc.teamcode.Constants.Globals.BlueGoalCorner;
import static org.firstinspires.ftc.teamcode.Constants.Globals.BlueGoalDistance;
import static org.firstinspires.ftc.teamcode.Constants.Globals.RedGoal;
import static org.firstinspires.ftc.teamcode.Constants.Globals.RedGoalCorner;
import static org.firstinspires.ftc.teamcode.Constants.Globals.RedGoalDistance;
import static org.firstinspires.ftc.teamcode.Constants.Globals.autoColor;
import static org.firstinspires.ftc.teamcode.Constants.OuttakeConstants.HoodConstants.maxAngle;
import static org.firstinspires.ftc.teamcode.Constants.OuttakeConstants.HoodConstants.minAngle;
import static org.firstinspires.ftc.teamcode.Constants.OuttakeConstants.LinkageCameraConstants.ArtifactPose;
import static org.firstinspires.ftc.teamcode.Constants.OuttakeConstants.ShooterConstants.deltaHeight;
import static org.firstinspires.ftc.teamcode.Constants.OuttakeConstants.ShooterConstants.gravitationalCoef;
import static org.firstinspires.ftc.teamcode.Constants.OuttakeConstants.ShooterConstants.scoreAngle;
import static org.firstinspires.ftc.teamcode.Constants.OuttakeConstants.ShooterConstants.tolerancePointRadius;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
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
    private final Telemetry telemetry;

    public Pose robotPose = new Pose(0,0,0);
    public final ElapsedTime timer = new ElapsedTime();
    public double distanceToGoal;
    public volatile boolean hasShot = false;
    private Thread breakBeamThread;
    private volatile boolean isRunning = true;
    public boolean isShooting = false;
    public static double shooterHeight = 11.8;
    public static double kE = 1.1;
    public static double baseClearanceOffset = 5.0;
    public double turretOffsetSotm = 0;
    public Hood hood;


    public Outtake(HardwareMap hardwareMap, Constants.Color color, Telemetry telemetry , boolean isAuto) {
        shooter = new Shooter(hardwareMap);
        turret = new Turret(hardwareMap, color, telemetry);
        hood = new Hood(hardwareMap , isAuto);
        linkageCamera = new LinkageCamera(hardwareMap,ArtifactPose,isAuto);
        blocker = new Blocker(hardwareMap, Blocker.ClosedPosition, isAuto);
        this.telemetry = telemetry;
    }

    public void startBreakBeamThread() {
    }

    public void stopBreakBeamThread() {
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
        targetAngleDegrees = Range.clip(targetAngleDegrees, minAngle, Constants.OuttakeConstants.HoodConstants.maxAngle);
        hood.setAngle(targetAngleDegrees);
        double targetHeightDelta = goal.targetZ - shooterHeight;
        double alphaRads = Math.toRadians(targetAngleDegrees);
        double sinA = Math.sin(alphaRads);
        double cosA = Math.cos(alphaRads);
        double denominator = 2 * cosA * (distanceToCenter * sinA - targetHeightDelta * cosA);
        if (denominator > 0) {
            double targetVelocity =  kE * distanceToCenter * Math.sqrt(gravitationalCoef / denominator)  * 0.0254;
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
    public Pose getGoalPose( ){
        switch (autoColor){
            case Blue:
                return BlueGoalCorner;
            default:
                return RedGoalCorner;
        }
    }
    public void shootOnTheMove(Vector robotVelocity){
        Vector robotToGoal = getGoalPose().minus(robotPose).getAsVector();
        double x = robotToGoal.getMagnitude() - tolerancePointRadius;

        double hoodAngle = Range.clip(Math.atan2(2*deltaHeight, x - Math.tan(scoreAngle)) , Math.toRadians(minAngle) , Math.toRadians(maxAngle))  * 0.0254;
        double v0 = Math.sqrt( (gravitationalCoef*x*x) / (2* Math.pow(Math.cos(hoodAngle), 2) * (x * Math.tan(hoodAngle) - deltaHeight)));

        double coordTheta = robotVelocity.getTheta() - robotToGoal.getTheta();

        double vrr = -Math.cos(coordTheta) * robotVelocity.getMagnitude();
        double vrt = Math.sin(coordTheta) * robotVelocity.getMagnitude();

        double vy0 = v0 * Math.sin(hoodAngle);
        double vxComp = v0 * Math.cos(hoodAngle) + vrr;
        double vxNew = Math.sqrt(vxComp * vxComp + vrt * vrt);
        double time = x / (Math.cos(hoodAngle) * v0);
        double xNew = vxNew * time;
        hoodAngle = Range.clip(Math.atan2(vy0, vxNew) , Math.toRadians(minAngle) , Math.toRadians(maxAngle));
        v0 = Math.sqrt( (gravitationalCoef*xNew*xNew) / (2* Math.pow(Math.cos(hoodAngle), 2) * (xNew * Math.tan(hoodAngle) - deltaHeight)))  * 0.0254;

        turretOffsetSotm = Math.tan(vrt / vxComp);
        hood.setAngleRadians(hoodAngle);
        shooter.setTargetVelocity(v0);
    }
    public String getShootOnTheMoveVelocity(Vector robotVelocity){
        Vector robotToGoal = getGoalPose().minus(robotPose).getAsVector();
        double x = robotToGoal.getMagnitude() - tolerancePointRadius;

        double hoodAngle = Range.clip(Math.atan2(2*deltaHeight, x - Math.tan(scoreAngle)) , Math.toRadians(minAngle) , Math.toRadians(maxAngle));
        double v0 = Math.sqrt( (gravitationalCoef*x*x) / (2* Math.pow(Math.cos(hoodAngle), 2) * (x * Math.tan(hoodAngle) - deltaHeight))) * 0.0254;

        double coordTheta = robotVelocity.getTheta() - robotToGoal.getTheta();

        double vrr = -Math.cos(coordTheta) * robotVelocity.getMagnitude();
        double vrt = Math.sin(coordTheta) * robotVelocity.getMagnitude();

        double vy0 = v0 * Math.sin(hoodAngle);
        double vxComp = v0 * Math.cos(hoodAngle) + vrr;
        double vxNew = Math.sqrt(vxComp * vxComp + vrt * vrt);
        double time = x / (Math.cos(hoodAngle) * v0);
        double xNew = vxNew * time;
        hoodAngle = Range.clip(Math.atan2(vy0, vxNew) , Math.toRadians(minAngle) , Math.toRadians(maxAngle));
        v0 = Math.sqrt( (gravitationalCoef*xNew*xNew) / (2* Math.pow(Math.cos(hoodAngle), 2) * (xNew * Math.tan(hoodAngle) - deltaHeight)))  * 0.0254;

        turretOffsetSotm = Math.tan(vrt / vxComp);
        return "V0 " + v0 + " Hood Angle " + hoodAngle;
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
    public void alignTurretSOTM(){
        turret.setTarget(turret.getTargetAngleFromDistance(robotPose) + turretOffsetSotm);
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
        telemetry.addData("Velocity", shooter.motorLeft.getCurrentVelocity());
        telemetry.addData("Target Velocity", shooter.getTarget());
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
