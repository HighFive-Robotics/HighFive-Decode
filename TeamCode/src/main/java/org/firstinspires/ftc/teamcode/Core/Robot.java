package org.firstinspires.ftc.teamcode.Core;

import static org.firstinspires.ftc.teamcode.Constants.Globals.autoColor;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Core.Hardware.HighModule;
import org.firstinspires.ftc.teamcode.Core.Module.Intake.IntakeMotor;
import org.firstinspires.ftc.teamcode.Core.Module.Intake.Intake;
import org.firstinspires.ftc.teamcode.Core.Module.Outtake.Shooter;

import java.util.List;

public class Robot extends HighModule {

    ElapsedTime timerShoot = new ElapsedTime(), timerIntake = new ElapsedTime() , voltageTimer = new ElapsedTime(), timerSorting = new ElapsedTime();

    Telemetry telemetry;
    public Actions lastAction = Actions.None;
    public Follower drive;
    public List<LynxModule> allHubs;
    protected HardwareMap hardwareMap;

    public Shooter shooter;
    public Intake intake;

    boolean isAuto;
    boolean stopShoot = false, stopIntake = false, stopSorting = false;

    private Constants.Color[] targetColors;
    private int sequenceIndex = 0;
    private boolean isColorSequenceRunning = false;
    private boolean isEmptySorterRunning = false;
    private int emptySorterSlot = 1;

    private enum SequenceState {
        ALIGNING,
        SHOOTING_START,
        SHOOTING_WAIT,
        FINISHED
    }
    private SequenceState sequenceState = SequenceState.FINISHED;

    public enum Actions {
        Shoot,
        PrepareForShooting,
        WaitToBeFedUp,
        NextSlot,
        PrevSlot,
        Slot1,
        Slot2,
        Slot3,
        FindGreen,
        FindPurple,
        EmptySorter,
        ShootPPG,
        ShootGPP,
        ShootPGP,
        None
    }

    public Robot(HardwareMap hardwareMap, Pose startPose, boolean isAuto, Constants.Color allianceColor, Telemetry telemetry , Gamepad gamepad) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        this.isAuto = isAuto;
        drive = Constants.createFollower(hardwareMap);
        if (isAuto) {
            drive.setStartingPose(startPose);
        } else {
            if(autoColor == Constants.Color.Blue) {
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

    public void setAction(Actions action){
        lastAction = action;

        if(action != Actions.EmptySorter && action != Actions.ShootPPG && action != Actions.ShootGPP && action != Actions.ShootPGP && action != Actions.Shoot){
            isColorSequenceRunning = false;
            isEmptySorterRunning = false;
        }

        switch (action){
            case Shoot:
                intake.setPower(IntakeMotor.States.Collect);
                stopShoot = true;
                timerShoot.reset();
                break;
            case PrepareForShooting:
                intake.setPower(IntakeMotor.States.Spit);
                stopIntake = true;
                timerIntake.reset();
                break;
            case NextSlot:
                prepareSorterMovement();
                intake.sorter.setNextSlot();
                break;
            case PrevSlot:
                prepareSorterMovement();
                intake.sorter.setPreviousSlot();
                break;
            case Slot1:
                prepareSorterMovement();
                intake.sorter.setSlot(1);
                break;
            case Slot2:
                prepareSorterMovement();
                intake.sorter.setSlot(2);
                break;
            case Slot3:
                prepareSorterMovement();
                intake.sorter.setSlot(3);
                break;
            case FindGreen:
                prepareSorterMovement();
                intake.setAction(Intake.IntakeActions.FindGreen);
                break;
            case FindPurple:
                prepareSorterMovement();
                intake.setAction(Intake.IntakeActions.FindPurple);
                break;
            case EmptySorter:
                isEmptySorterRunning = true;
                emptySorterSlot = 1;
                sequenceState = SequenceState.ALIGNING;
                prepareSorterMovement();
                intake.sorter.setSlot(1);
                break;
            case ShootPPG:
                startColorSequence(Constants.Color.Purple, Constants.Color.Purple, Constants.Color.Green);
                break;
            case ShootGPP:
                startColorSequence(Constants.Color.Green, Constants.Color.Purple, Constants.Color.Purple);
                break;
            case ShootPGP:
                startColorSequence(Constants.Color.Purple, Constants.Color.Green, Constants.Color.Purple);
                break;
        }
    }

    private void prepareSorterMovement(){
        intake.setPower(IntakeMotor.States.Collect);
        stopSorting = true;
        timerSorting.reset();
    }

    private void startColorSequence(Constants.Color... colors) {
        targetColors = colors;
        sequenceIndex = 0;
        isColorSequenceRunning = true;
        isEmptySorterRunning = false;
        sequenceState = SequenceState.ALIGNING;
        findAndAlignToColor(targetColors[0]);
    }

    private void findAndAlignToColor(Constants.Color color) {
        int targetSlot = -1;
        for (int i = 1; i <= 3; i++) {
            if (intake.sorter.getColor(i) == color) {
                targetSlot = i;
                break;
            }
        }

        if (targetSlot != -1) {
            prepareSorterMovement();
            intake.sorter.setSlot(targetSlot);
        }
    }

    public boolean isDone() {
        return !drive.isBusy();
    }

    @Override
    public void update() {
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
        if(voltageTimer.milliseconds() >= 500) {
            Constants.Globals.voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
            voltageTimer.reset();
        }
        if(stopShoot && (timerShoot.milliseconds() > 1200 || (timerShoot.milliseconds() > 200 && shooter.getVelocityError() >= 0.7))){
            stopShoot = false;
            intake.setPower(IntakeMotor.States.Wait);
        }
        if(stopIntake && timerIntake.milliseconds() >= 450){
            stopIntake = false;
            intake.setPower(IntakeMotor.States.Wait);
        }
        if(stopSorting && (timerSorting.milliseconds() > 500 || (timerSorting.milliseconds() > 100 && intake.sorter.atTarget()))){
            stopSorting = false;
            intake.setPower(IntakeMotor.States.Wait);
        }

        updateSequences();

        shooter.update();
        intake.update();
        drive.update();

    }

    private void updateSequences() {
        if (isEmptySorterRunning) {
            switch (sequenceState) {
                case ALIGNING:
                    if ((timerSorting.milliseconds() > 100 && intake.sorter.atTarget()) || timerSorting.milliseconds() > 600) {
                        sequenceState = SequenceState.SHOOTING_START;
                    }
                    break;
                case SHOOTING_START:
                    setAction(Actions.Shoot);
                    sequenceState = SequenceState.SHOOTING_WAIT;
                    break;
                case SHOOTING_WAIT:
                    if (!stopShoot) {
                        intake.sorter.setColor(Constants.Color.None, intake.sorter.getSlot());

                        emptySorterSlot++;
                        if (emptySorterSlot > 3) {
                            isEmptySorterRunning = false;
                            sequenceState = SequenceState.FINISHED;
                            lastAction = Actions.None;
                        } else {
                            prepareSorterMovement();
                            intake.sorter.setSlot(emptySorterSlot);
                            sequenceState = SequenceState.ALIGNING;
                        }
                    }
                    break;
            }
        } else if (isColorSequenceRunning) {
            switch (sequenceState) {
                case ALIGNING:
                    boolean colorFound = false;
                    for (int i = 1; i <= 3; i++) {
                        if (intake.sorter.getColor(i) == targetColors[sequenceIndex]) {
                            colorFound = true;
                            break;
                        }
                    }

                    if (!colorFound) {
                        sequenceIndex++;
                        if (sequenceIndex >= targetColors.length) {
                            isColorSequenceRunning = false;
                            sequenceState = SequenceState.FINISHED;
                            lastAction = Actions.None;
                        } else {
                            findAndAlignToColor(targetColors[sequenceIndex]);
                        }
                    } else if ((timerSorting.milliseconds() > 100 && intake.sorter.atTarget()) || timerSorting.milliseconds() > 600) {
                        sequenceState = SequenceState.SHOOTING_START;
                    }
                    break;
                case SHOOTING_START:
                    setAction(Actions.Shoot);
                    sequenceState = SequenceState.SHOOTING_WAIT;
                    break;
                case SHOOTING_WAIT:
                    if (!stopShoot) {
                        intake.sorter.setColor(Constants.Color.None, intake.sorter.getSlot());

                        sequenceIndex++;
                        if (sequenceIndex >= targetColors.length) {
                            isColorSequenceRunning = false;
                            sequenceState = SequenceState.FINISHED;
                            lastAction = Actions.None;
                        } else {
                            findAndAlignToColor(targetColors[sequenceIndex]);
                            sequenceState = SequenceState.ALIGNING;
                        }
                    }
                    break;
            }
        }
    }
}