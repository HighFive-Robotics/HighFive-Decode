package org.firstinspires.ftc.teamcode.Core.Module.Outtake;


import static org.firstinspires.ftc.teamcode.Constants.DeviceNames.breakBeamOuttakeName;
import static org.firstinspires.ftc.teamcode.Constants.Globals.BlueGoalDistance;
import static org.firstinspires.ftc.teamcode.Constants.Globals.RedGoalDistance;
import static org.firstinspires.ftc.teamcode.Constants.Globals.autoColor;
import static org.firstinspires.ftc.teamcode.Constants.OuttakeConstants.ShooterBackWheelParams.kC;

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
    public Blocker blocker;
    public DigitalChannel breakBeamOuttake;
    private final Telemetry telemetry;

    public Pose robotPose;
    public final ElapsedTime timer = new ElapsedTime();
    public double distanceToGoal;
    public volatile boolean hasShot = false;
    private Thread breakBeamThread;
    private volatile boolean isRunning = true;
    public boolean isShooting = false;


    public Outtake(HardwareMap hardwareMap, Constants.Color color, Telemetry telemetry) {
        shooter = new Shooter(hardwareMap);
        turret = new Turret(hardwareMap, color, telemetry);
        blocker = new Blocker(hardwareMap, Blocker.ClosedPosition, true);
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

    public void setShootingVelocityCompensation(double dist) {
        shooter.setTargetVelocityCompensation(dist);
    }

    public void setShootingVelocity(double up, double down) {
        shooter.setDownTargetVelocity(down);
        shooter.setUpTargetVelocity(up);
    }

    public void setShootingVelocityCompensation() {
        setShootingVelocityCompensation(distanceToGoal);
    }

    public void setShootingVelocityOffset(double offset) {
        setShootingVelocity(distanceToGoal + offset);
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

    public boolean atTargetCompensated() {
        return shooter.atTargetCompensated() && turret.atTarget();
    }

    public void increaseToleranceOffset(double down, double up) {
        shooter.addToDownToleranceOffset(down);
        shooter.addToUpToleranceOffset(up);
    }

    public void decreaseToleranceOffset(double down, double up) {
        shooter.addToDownToleranceOffset(-down);
        shooter.addToUpToleranceOffset(-up);
    }


    public void setToleranceCompensationOffset(double offset) {
        shooter.setToleranceCompensationOffset(offset);
    }

    public void setToleranceOffset(double down, double up) {
        shooter.setDownToleranceOffset(down);
        shooter.setUpToleranceOffset(up);
    }

    public boolean checkErrorTolerance(double down, double up) {
        return shooter.getVelocityErrorDown() >= down || shooter.getVelocityErrorUp() >= up;
    }

    public boolean checkErrorToleranceDown(double error) {
        return shooter.getVelocityErrorDown() >= error;
    }

    public boolean checkErrorToleranceUp(double error) {
        return shooter.getVelocityErrorUp() >= error;
    }

    public boolean checkErrorToleranceCompensation(double error) {
        return Math.abs(shooter.getVelocityErrorCompensation()) <= error;
    }

    public void addErrorToleranceScaled() {
        double offset;
        if(distanceToGoal <= 140){
            offset = Range.clip(Range.scale(distanceToGoal, 60, 140, 0.45, 0.25), 0.15, 0.45);
        }else{
            offset = Range.clip(Range.scale(distanceToGoal, 140, 360, 0.25, 0.15), 0.15, 0.25);
        }
        shooter.addToleranceCompensationOffset(offset);
    }

    public boolean detectShoot() {
        if (atTargetCompensated()) {
            shooter.wasAtTarget = true;
            return false;
        }
        double jerkOffset = Range.clip(Range.scale(distanceToGoal, 60, 360, 0.15, 0.3), 0.15, 0.3);
        if (shooter.wasAtTarget && (shooter.jerk >= jerkOffset)) {
            shooter.wasAtTarget = false;
            return true;
        }
        return false;
    }

    public void resetErrorTolerance() {
        shooter.setToleranceCompensationOffset(0);
        setToleranceOffset(0, 0);
    }

    public double getDistanceToGoalTime() {
        return distanceToGoal <= 240 ? 275 : 375;
    }

    @Override
    public void update() {
        shooter.update();
        turret.update();
        blocker.update();
//        if (hasShot) {
//            if (timer.milliseconds() >= 30 || !breakBeamOuttake.getState()) {
//                hasShot = false;
//            }
//        }
//
//        if (!hasShot && breakBeamOuttake.getState()) {
//            timer.reset();
//            hasShot = true;
//        }
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

    public void debug() {
        telemetry.addData("Shooter Down Tollerence + Offset", shooter.getDownOffset());
        telemetry.addData("Shooter Up Tollerence + Offset", shooter.getUpOffset());
        telemetry.addData("Shooter Compensation Tolerance + Offset", shooter.getToleranceCompensation());
        telemetry.addData("Distance ", distanceToGoal);
        telemetry.addData("Kc ", kC);
        telemetry.addData("Down Velocity", shooter.motorDown.getCurrentVelocity());
        telemetry.addData("Down Target", shooter.getTargetDown());
        telemetry.addData("Up Velocity", shooter.motorUp.getCurrentVelocity());
        telemetry.addData("Up Target", shooter.getTargetUp());
        telemetry.addData("General Target", shooter.target);
        telemetry.addData("Down error velocity", shooter.getVelocityErrorDown());
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
