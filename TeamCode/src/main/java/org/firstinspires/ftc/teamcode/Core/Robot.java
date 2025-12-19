package org.firstinspires.ftc.teamcode.Core;

import static org.firstinspires.ftc.teamcode.Constants.Color.Green;
import static org.firstinspires.ftc.teamcode.Constants.Color.None;
import static org.firstinspires.ftc.teamcode.Constants.Color.Purple;
import static org.firstinspires.ftc.teamcode.Constants.Globals.autoColor;
import static org.firstinspires.ftc.teamcode.Constants.Intake.SorterConstants.artifactNumber;

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
    ElapsedTime timerShoot = new ElapsedTime(), timerIntake = new ElapsedTime(), voltageTimer = new ElapsedTime(), intakeHelper = new ElapsedTime(), alignTimer = new ElapsedTime();
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
    boolean intakeHelping = false;
    public boolean isManualControl = true;
    public boolean shouldAutoCycle = false;
    SortingBehaviour shootingSeq = SortingBehaviour.Idle;
    int slotCycles = -1;
    Constants.Color[] motif = {None, None, None};

    public enum SortingBehaviour {
        Aligning,
        Shooting,
        Sorting,
        Idle,

    }

    public enum Actions {
        Shoot,
        PrepareForShooting,
        NextSlot,
        PrevSlot,
        FindGreen,
        FindPurple,
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
                shouldAutoCycle = false;
                isManualControl = false;
                timerShoot.reset();
                break;
            case PrepareForShooting:
                intake.intakeMotor.setPower(-0.7);
                stopIntake = true;
                shouldAutoCycle = false;
                isManualControl = false;
                timerIntake.reset();
                break;
            case PrevSlot:
                setIntakeForSwitch();
                isManualControl = false;
                shouldAutoCycle = false;
                intake.sorter.setPreviousSlot();
                break;
            case NextSlot:
                setIntakeForSwitch();
                isManualControl = false;
                shouldAutoCycle = false;
                intake.sorter.setNextSlot();
                break;
            case FindGreen:
                setIntakeForSwitch();
                isManualControl = false;
                shouldAutoCycle = false;
                intake.findColor(Intake.FindColors.FindGreen);
                break;
            case FindPurple:
                setIntakeForSwitch();
                isManualControl = false;
                shouldAutoCycle = false;
                intake.findColor(Intake.FindColors.FindPurple);
                break;
            case EmptySorter:
                alignTimer.reset();
                isManualControl = false;
                shouldAutoCycle = false;
                slotCycles = 1;
                motif = new Constants.Color[]{None, None, None};
                intake.sorter.setSlot(intake.sorter.getSlot());
                shootingSeq = SortingBehaviour.Aligning;
                break;
            case ShootGPP:
                alignTimer.reset();
                isManualControl = false;
                shouldAutoCycle = false;
                slotCycles = 1;
                motif = new Constants.Color[]{Green, Purple, Purple};
                findSpecificColor(motif[0]);
                shootingSeq = SortingBehaviour.Aligning;
                break;
            case ShootPGP:
                alignTimer.reset();
                isManualControl = false;
                shouldAutoCycle = false;
                slotCycles = 1;
                motif = new Constants.Color[]{Purple, Green, Purple};
                findSpecificColor(motif[0]);
                shootingSeq = SortingBehaviour.Aligning;
                break;
            case ShootPPG:
                alignTimer.reset();
                isManualControl = false;
                shouldAutoCycle = false;
                slotCycles = 1;
                motif = new Constants.Color[]{Purple, Purple, Green};
                findSpecificColor(motif[0]);
                shootingSeq = SortingBehaviour.Aligning;
                break;

        }
    }
    public void setIntakeForSwitch(){
        intake.setState(Intake.States.Wait);
        intake.setPower(IntakeMotor.States.Collect);
        intakeHelping = true;
        intakeHelper.reset();
    }
    public void findSpecificColor(Constants.Color color) {
        switch (color) {
            case Green:
                if (Constants.Intake.SorterConstants.greenArtifactNumber > 0) {
                    setAction(Actions.FindGreen);
                } else {
                    setAction(Actions.NextSlot);
                }
                break;
            case Purple:
                if (Constants.Intake.SorterConstants.purpleArtifactNumber > 0) {
                    setAction(Actions.FindPurple);
                } else {
                    setAction(Actions.NextSlot);
                }
                break;
            case None:
                setAction(Actions.NextSlot);
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
            intake.sorter.setColor(None, intake.sorter.getSlot());
            stopShoot = false;
            intake.setState(Intake.States.Wait);
            intake.setPower(IntakeMotor.States.Wait);
        }
        if (stopIntake && timerIntake.milliseconds() >= 185) {
            stopIntake = false;
            intake.setState(Intake.States.Wait);
            intake.setPower(IntakeMotor.States.Wait);
        }
        if (intakeHelping && intakeHelper.milliseconds() >= 415) {
            intake.setPower(IntakeMotor.States.Wait);
            intakeHelping = false;
        }
        intake.update();
        if (shouldAutoCycle) {
            switch (intake.getCollectType()) {
                case Sorted:
                    if (intake.sorter.getState() == Sorter.States.Automated) {
                        if (!intake.sorter.isFull) {
                            if (intake.currentColor != None && !intake.artifactPassThrough) {
                                setAction(Actions.NextSlot);
                            }
                        }
                        intake.updateColor();
                    }
                    break;
                case Mix:
                    if (intake.sorter.getState() == Sorter.States.Automated && artifactNumber < 1) {
                        if (!intake.sorter.isFull) {
                            if (intake.currentColor != None && !intake.artifactPassThrough) {
                                setAction(Actions.NextSlot);
                            }
                        }
                        intake.updateColor();
                    }
                    break;
                case Normal:
                    break;
            }
        }
        if (shootingSeq != SortingBehaviour.Idle) {
            switch (shootingSeq) {
                case Aligning:
                    if (intake.atTarget() || alignTimer.milliseconds() >= 400) {
                        shootingSeq = SortingBehaviour.Shooting;
                    }
                    break;
                case Shooting:
                        setAction(Actions.Shoot);
                        shootingSeq = SortingBehaviour.Sorting;
                        break;
                case Sorting:
                    if (slotCycles <= 2) {
                        if (!stopShoot) {
                            findSpecificColor(motif[slotCycles]);
                            alignTimer.reset();
                            shootingSeq = SortingBehaviour.Aligning;
                            slotCycles++;
                        }
                    } else {
                        shootingSeq = SortingBehaviour.Idle;
                        slotCycles = -1;
                    }
            }
        }
        shooter.update();
        drive.update();
        telemetry.addData("Timer", intakeHelper.milliseconds());
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
    }
}