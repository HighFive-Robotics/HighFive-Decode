package org.firstinspires.ftc.teamcode.Core;

import static org.firstinspires.ftc.teamcode.Constants.Color.None;
import static org.firstinspires.ftc.teamcode.Constants.Globals.BlueGoal;
import static org.firstinspires.ftc.teamcode.Constants.Globals.RedGoal;
import static org.firstinspires.ftc.teamcode.Constants.Globals.autoColor;
import static org.firstinspires.ftc.teamcode.Constants.Intake.SorterConstants.artifactNumber;
import static org.firstinspires.ftc.teamcode.Core.Module.Intake.Intake.Actions.NextSlot;
import static org.firstinspires.ftc.teamcode.Core.Module.Outtake.BlockerOuttake.States.Close;
import static org.firstinspires.ftc.teamcode.Core.Module.Outtake.BlockerOuttake.States.Open;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Core.Hardware.HighCamera;
import org.firstinspires.ftc.teamcode.Core.Hardware.HighModule;
import org.firstinspires.ftc.teamcode.Core.Module.Intake.IntakeMotor;
import org.firstinspires.ftc.teamcode.Core.Module.Intake.Intake;

import org.firstinspires.ftc.teamcode.Core.Module.Intake.Sorter;
import org.firstinspires.ftc.teamcode.Core.Module.Outtake.Shooter;

import java.util.List;

public class Robot extends HighModule {
    ElapsedTime timerShoot = new ElapsedTime(), timerIntake = new ElapsedTime(), voltageTimer = new ElapsedTime(), intakeHelper = new ElapsedTime();
    Telemetry telemetry;
    public Actions lastAction = Actions.None;
    public Follower drive;
    public List<LynxModule> allHubs;
    protected HardwareMap hardwareMap;
    public Shooter shooter;
    public Intake intake;
    public HighCamera camera;
    boolean isAuto;
    boolean stopShoot = false, stopIntake = false, startShootingSequence = false;
    int shootingState = 0;
    Constants.Color allianceColor;

    public enum Actions {
        Shoot,
        PrepareForShooting,
        ShootSequence,
        EmptySorter,
        ShootPPG,
        ShootGPP,
        ShootPGP,
        None
    }

    public Robot(HardwareMap hardwareMap, Pose startPose, boolean isAuto, Constants.Color allianceColor, Telemetry telemetry, Gamepad gamepad) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        this.isAuto = isAuto;
        this.allianceColor = allianceColor;
        drive = Constants.createFollower(hardwareMap);
        camera = new HighCamera(hardwareMap, HighCamera.Pipelines.AprilTagId);
        if (isAuto) {
            drive.setStartingPose(startPose);
        } else {
            if (autoColor == Constants.Color.Blue) {
                drive.startFieldCentricDrive(gamepad, true, Math.PI + startPose.getHeading());
            } else {
                drive.startFieldCentricDrive(gamepad, true, startPose.getHeading());
            }
        }
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    public Robot(HardwareMap hardwareMap, Pose startPose, boolean isAuto, Constants.Color allianceColor, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        this.isAuto = isAuto;
        this.allianceColor = allianceColor;
        drive = Constants.createFollower(hardwareMap);
        camera = new HighCamera(hardwareMap, HighCamera.Pipelines.AprilTagId);
        if (isAuto) {
            drive.setStartingPose(startPose);
        }
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    public void setAction(Actions action) {
        lastAction = action;
        switch (action) {
            case Shoot:
                intake.setPower(IntakeMotor.States.Collect);
                stopShoot = true;
                intake.canStop = false;
                timerShoot.reset();
                break;
            case PrepareForShooting:
                intake.intakeMotor.setPower(-0.7);
                intake.canStop = false;
                stopIntake = true;
                timerIntake.reset();
                break;
            case ShootSequence:
                startShootingSequence = true;
                shootingState = 0;
            case EmptySorter:
                break;
            case ShootGPP:
                break;
            case ShootPGP:
                break;
            case ShootPPG:
                break;

        }
    }

    public boolean isDone() {
        return !drive.isBusy();
    }

    @Override
    public void update() {
        if (voltageTimer.milliseconds() >= 500) {
            Constants.Globals.voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
            voltageTimer.reset();
        }
        if (stopShoot && (timerShoot.milliseconds() > 800 || (timerShoot.milliseconds() > 125 && shooter.getVelocityError() >= 0.7))) {
            intake.sorter.setColor(None, intake.currentSlot);
            stopShoot = false;
            if(!intake.helpingSorter){
                intake.setPower(IntakeMotor.States.Wait);
            }
            intake.canStop = true;
            intake.setState(Intake.States.Wait);
        }
        if (stopIntake && timerIntake.milliseconds() >= 185) {
            stopIntake = false;
            if(!intake.helpingSorter){
                intake.setPower(IntakeMotor.States.Wait);
            }
            intake.canStop = true;
            intake.setState(Intake.States.Wait);
        }
        if(startShootingSequence){
            intake.setState(Intake.States.Wait);
            switch (shootingState){
                case 0:
                    if(intake.currentColor != None){
                        if(intake.atTarget()){
                            shooter.blocker.setState(Open,250);
                            shootingState = 1;
                        }
                    } else {
                        shootingState = 2;
                    }
                    break;
                case 1:
                    if(shooter.blocker.atTarget()){
                        setAction(Actions.Shoot);
                        shootingState = 2;
                    }
                    break;
                case 2:
                    if(artifactNumber == 0){
                        startShootingSequence = false;
                        intake.setState(Intake.States.Collect);
                    }
                    else if(intake.currentColor == None){
                        shooter.blocker.setState(Close,250);
                        intake.setAction(NextSlot);
                        shootingState = 0;
                    }
                    break;
            }
        }
        if (intake.getState() == Intake.States.Collect) {
            switch (intake.getCollectType()) {
                case Sorted:
                    if (intake.sorter.getState() == Sorter.States.Automated) {
                        if (!intake.sorter.isFull) {
                            if (intake.currentColor != None && !intake.artifactPassThrough) {
                                intake.setAction(NextSlot);
                            }
                        }
                        intake.updateColor();
                    }
                    break;
                case Mix:
                    if (intake.sorter.getState() == Sorter.States.Automated && artifactNumber < 1) {
                        if (!intake.sorter.isFull) {
                            if (intake.currentColor != None && !intake.artifactPassThrough) {
                                intake.setAction(NextSlot);
                            }
                        }
                        intake.updateColor();
                    }
                    break;
                case Normal:
                    break;
            }
        }

        intake.update();
        shooter.update();
        drive.update();
        telemetry.addData("Timer", intakeHelper.milliseconds());
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
    }

    public double getDistance(){
        switch (allianceColor){
            case Blue:{
                return 2.54 * Math.hypot(BlueGoal.getX() - drive.getPose().getX(),BlueGoal.getY() - drive.getPose().getY());
            }
            default:{
                return 2.54 * Math.hypot(RedGoal.getX() - drive.getPose().getX(),RedGoal.getY() - drive.getPose().getY());
            }
            case None:
                return -1;
        }
    }
}