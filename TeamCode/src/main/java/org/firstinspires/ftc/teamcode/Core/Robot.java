package org.firstinspires.ftc.teamcode.Core;

import static org.firstinspires.ftc.teamcode.Constants.Color.None;
import static org.firstinspires.ftc.teamcode.Constants.Globals.BlueGoal;
import static org.firstinspires.ftc.teamcode.Constants.Globals.RedGoal;
import static org.firstinspires.ftc.teamcode.Constants.Globals.autoColor;
import static org.firstinspires.ftc.teamcode.Constants.Intake.SorterConstants.artifactNumber;
import static org.firstinspires.ftc.teamcode.Constants.Intake.SorterConstants.greenArtifactNumber;
import static org.firstinspires.ftc.teamcode.Constants.Intake.SorterConstants.purpleArtifactNumber;
import static org.firstinspires.ftc.teamcode.Constants.Intake.SorterConstants.targetColors;
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

import java.util.Arrays;
import java.util.List;

public class Robot extends HighModule {

    ElapsedTime timerShoot = new ElapsedTime(), timerIntake = new ElapsedTime(), voltageTimer = new ElapsedTime(), intakeHelper = new ElapsedTime(), sorterTimer = new ElapsedTime();
    Telemetry telemetry;
    public Actions lastAction = Actions.None;
    public Follower drive;
    public List<LynxModule> allHubs;
    protected HardwareMap hardwareMap;
    public Shooter shooter;
    public Intake intake;
    public HighCamera camera;
    boolean isAuto;
    boolean stopShoot = false, stopIntake = false;
    public boolean startShootingSequence = false, startShootingSequenceQueue = false;
    int shootingState = 0, shootingStateQueue = 0;
    Constants.Color allianceColor;
    public Constants.Color[] colorsQueue = {Constants.Color.None, Constants.Color.None, Constants.Color.None};
    public int colorsQueueIndex = 0;

    public enum Actions {
        Shoot,
        PrepareForShooting,
        ShootFast,
        EmptySorter,
        StopShooting,
        AddPurpleToQueue,
        AddGreenToQueue,
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
            case ShootFast:
                startShootingSequence = true;
                startShootingSequenceQueue = false;
                shootingState = 0;
                sorterTimer.reset();
                break;
            case StopShooting:
                startShootingSequence = false;
                startShootingSequenceQueue = false;
                intake.setPower(IntakeMotor.States.Wait);
                intake.setState(Intake.States.Collect);
                break;
            case AddPurpleToQueue:
                if(artifactNumber > 0 && colorsQueueIndex < artifactNumber){
                    if(purpleArtifactNumber > 0){
                        colorsQueue[colorsQueueIndex] = Constants.Color.Purple;
                        startShootingSequenceQueue = true;
                        shootingStateQueue = 0;
                        sorterTimer.reset();
                        colorsQueueIndex++;
                    }
                }
                break;
            case AddGreenToQueue:
                if(artifactNumber > 0 && colorsQueueIndex < artifactNumber){
                    if(greenArtifactNumber > 0){
                        colorsQueue[colorsQueueIndex] = Constants.Color.Green;
                        startShootingSequenceQueue = true;
                        shootingStateQueue = 0;
                        sorterTimer.reset();
                        colorsQueueIndex++;
                    }
                }
                break;
            case ShootGPP:
                if(artifactNumber == 3 && greenArtifactNumber == 1 && purpleArtifactNumber == 2){
                    setFullQueue(targetColors[0]);
                    startShootingSequenceQueue = true;
                    startShootingSequence = false;
                    shootingStateQueue = 0;
                    sorterTimer.reset();
                }
                break;
            case ShootPGP:
                if(artifactNumber == 3 && greenArtifactNumber == 1 && purpleArtifactNumber == 2) {
                    setFullQueue(targetColors[1]);
                    startShootingSequenceQueue = true;
                    startShootingSequence = false;
                    shootingStateQueue = 0;
                    sorterTimer.reset();
                }
                break;
            case ShootPPG:
                if(artifactNumber == 3 && greenArtifactNumber == 1 && purpleArtifactNumber == 2) {
                    setFullQueue(targetColors[2]);
                    startShootingSequenceQueue = true;
                    startShootingSequence = false;
                    shootingStateQueue = 0;
                    sorterTimer.reset();
                }
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
        if (stopShoot && (timerShoot.milliseconds() > 900 || (timerShoot.milliseconds() > 125 && shooter.getVelocityError() >= 0.7))) {
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
                        if(intake.atTarget() || sorterTimer.milliseconds() >= 30){
                            shooter.blocker.setState(Open,50);
                            shootingState = 1;
                        }
                    } else {
                        shootingState = 2;
                    }
                    break;
                case 1:
                    if(shooter.blocker.atTarget() && shooter.atTarget()){
                        setAction(Actions.Shoot);
                        shootingState = 2;
                    }
                    break;
                case 2:
                    if(intake.currentColor == None){
                        shooter.blocker.setState(Close, 50);
                        shootingState = 3;
                    }
                    break;
                case 3:
                    if(shooter.blocker.atTarget()){
                        intake.setAction(NextSlot);
                        shootingState = 0;
                        sorterTimer.reset();
                    }
                    break;
            }
            if(artifactNumber == 0){
                startShootingSequence = false;
                intake.setState(Intake.States.Collect);
            }
        }

        telemetry.addData("startShootingSequenceQueue:", startShootingSequenceQueue);
        telemetry.addData("shootingStateQueue:", shootingStateQueue);
        telemetry.addData("colorsQueue:", Arrays.toString(colorsQueue));
        telemetry.addData("Current color:", intake.currentColor);
        telemetry.addData("artifactNumber:", artifactNumber);
        telemetry.addData("purpleArtifactNumber:", purpleArtifactNumber);
        telemetry.addData("greenArtifactNumber:", greenArtifactNumber);
        telemetry.addData("colors index:", colorsQueueIndex);
        if(startShootingSequenceQueue){
            intake.setState(Intake.States.Wait);
            switch (shootingStateQueue){
                case 0:
                    if(intake.currentColor == colorsQueue[0]){
                        if(intake.atTarget() || sorterTimer.milliseconds() >= 350){
                            shooter.blocker.setState(Open,50);
                            shootingStateQueue = 1;
                        }
                    } else {
                        shootingStateQueue = 4;
                    }
                    break;
                case 1:
                    if(shooter.blocker.atTarget() && shooter.atTarget()){
                        setAction(Actions.Shoot);
                        colorsQueue[0] = colorsQueue[1];
                        colorsQueue[1] = colorsQueue[2];
                        colorsQueue[2] = None;
                        colorsQueueIndex = Math.max(0, colorsQueueIndex - 1);

                        shootingStateQueue = 2;
                    }
                    break;
                case 2:
                    if(intake.currentColor == None){
                        shooter.blocker.setState(Close, 50);
                        shootingStateQueue = 3;
                    }
                    break;
                case 3:
                    if(shooter.blocker.atTarget()){
                        if(colorsQueue[0] != None) {
                            intake.findColor(colorsQueue[0]);
                        }
                        shootingStateQueue = 0;
                        sorterTimer.reset();
                    }
                    break;
                case 4:
                    if(colorsQueue[0] != None){
                        intake.findColor(colorsQueue[0]);
                    }
                    shootingStateQueue = 0;
                    sorterTimer.reset();
                    break;
            }
            if(artifactNumber == 0 || (colorsQueueIndex == 0 && shootingStateQueue == 0 && intake.currentColor == None)){
                startShootingSequenceQueue = false;
                intake.setState(Intake.States.Collect);
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

    private void setFullQueue(Constants.Color[] color){
        colorsQueue[0] = color[0];
        colorsQueue[1] = color[1];
        colorsQueue[2] = color[2];
        colorsQueueIndex = 3;
    }

    public double getDistance(){
        switch (allianceColor){
            case Blue:{
                return 2.54 * Math.hypot(BlueGoal.getX() - drive.getPose().getX(),BlueGoal.getY() - drive.getPose().getY());
            }
            case Red:{
                return 2.54 * Math.hypot(RedGoal.getX() - drive.getPose().getX(),RedGoal.getY() - drive.getPose().getY());
            }
        }
        return -1;
    }
    public boolean isSorterEmpty(){
        return intake.sorter.isEmpty;
    }
}