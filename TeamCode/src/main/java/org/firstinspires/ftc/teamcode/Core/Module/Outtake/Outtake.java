package org.firstinspires.ftc.teamcode.Core.Module.Outtake;


import static org.firstinspires.ftc.teamcode.Constants.DeviceNames.breakBeamOuttakeName;
import static org.firstinspires.ftc.teamcode.Constants.Globals.BlueGoalDistance;
import static org.firstinspires.ftc.teamcode.Constants.Globals.RedGoalDistance;
import static org.firstinspires.ftc.teamcode.Constants.Globals.autoColor;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    private double distanceToGoal;
    public boolean hasShot = false;


    public Outtake(HardwareMap hardwareMap, Constants.Color color, Telemetry telemetry) {
        shooter = new Shooter(hardwareMap);
        turret = new Turret(hardwareMap, color, telemetry);
        blocker = new Blocker(hardwareMap, Blocker.ClosedPosition, true);
        breakBeamOuttake = hardwareMap.get(DigitalChannel.class, breakBeamOuttakeName);
        breakBeamOuttake.setMode(DigitalChannel.Mode.INPUT);
        this.telemetry = telemetry;
    }

    public void setShootingVelocity(double dist) {
        shooter.setTargetVelocity(dist);
    }

    public void setShootingVelocityCompensation(double dist) {
        shooter.setTargetVelocityCompensation(dist);
    }

    public void setShootingVelocity(double up, double down) {
        shooter.setDownTargetVelocity(down);
        shooter.setUpTargetVelocity(up);
    }

    public void setShootingVelocity() {
        setShootingVelocity(distanceToGoal);
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

    public boolean atTarget() {
        return shooter.atTarget() && turret.atTarget();
    }

    public void increaseToleranceOffset(double down, double up) {
        shooter.addToDownToleranceOffset(down);
        shooter.addToUpToleranceOffset(up);
    }

    public void decreaseToleranceOffset(double down, double up) {
        shooter.addToDownToleranceOffset(-down);
        shooter.addToUpToleranceOffset(-up);
    }

    public void setToleranceOffset(double down, double up) {
        shooter.setDownToleranceOffset(down);
        shooter.setUpToleranceOffset(up);
    }

    public boolean checkErrorTolerance(double down, double up) {
        return shooter.getVelocityErrorDown() >= down || shooter.getVelocityErrorUp() >= up;
    }

    @Override
    public void update() {
        shooter.update();
        turret.update();
        blocker.update();

        if (breakBeamOuttake.getState() && timer.milliseconds() >= 30) {
            timer.reset();
            hasShot = true;
        }

        if (hasShot) {
            if (timer.milliseconds() >= 30) {
                hasShot = false;
                timer.reset();
            }
        }
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
