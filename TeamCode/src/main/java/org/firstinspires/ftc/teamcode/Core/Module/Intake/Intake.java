package org.firstinspires.ftc.teamcode.Core.Module.Intake;

import static org.firstinspires.ftc.teamcode.Constants.Color.Green;
import static org.firstinspires.ftc.teamcode.Constants.Color.None;
import static org.firstinspires.ftc.teamcode.Constants.Color.Purple;
import static org.firstinspires.ftc.teamcode.Constants.DeviceNames.breakBeamIntakeName;
import static org.firstinspires.ftc.teamcode.Constants.DeviceNames.intakeSensorName;
import static org.firstinspires.ftc.teamcode.Constants.Intake.SorterConstants.artifactNumber;
import static org.firstinspires.ftc.teamcode.Constants.Intake.SorterConstants.greenArtifactNumber;
import static org.firstinspires.ftc.teamcode.Constants.Intake.SorterConstants.purpleArtifactNumber;
import static org.firstinspires.ftc.teamcode.Constants.Intake.SorterConstants.sorterColors;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Core.Hardware.HighModule;
import org.firstinspires.ftc.teamcode.Core.Hardware.HighSensor;

public class Intake extends HighModule {
    public IntakeMotor intakeMotor;
    public Sorter sorter;
    public HighSensor sensor;
    public DigitalChannel breakBeam;

    public Sorter.Slots currentSlot;
    public Constants.Color currentColor;
    public int currentSlotNumber;

    ElapsedTime timer = new ElapsedTime(),intakeHelper = new ElapsedTime();

    public boolean artifactPassThrough = false;
    public boolean breakBeamCollected = false, helpingSorter = false;
    private boolean colorAssignedToCurrentSample = false;
    public boolean canStop = true;

    States state = States.Collect;
    IntakeMotor.States lastPower = IntakeMotor.States.Wait;
    CollectTypes collectType = CollectTypes.Sorted;
    Actions currentAction;

    public enum States {
        Collect,
        Wait
    }

    public enum Actions{
        FindGreen,
        FindPurple,
        NextSlot,
        PreviousSlot
    }

    public enum CollectTypes {
        Sorted,
        Normal,
        Mix
    }

    public Intake(HardwareMap hwMap){
        intakeMotor = new IntakeMotor(hwMap);
        sorter = new Sorter(hwMap,intakeMotor.motor.motor,0);
        sensor = new HighSensor(hwMap, intakeSensorName);
        breakBeam = hwMap.get(DigitalChannel.class, breakBeamIntakeName);
        breakBeam.setMode(DigitalChannel.Mode.INPUT);
    }

    public Intake(HardwareMap hwMap, Sorter.Slots slot){
        intakeMotor = new IntakeMotor(hwMap);
        sorter = new Sorter(hwMap,intakeMotor.motor.motor,0);
        sensor = new HighSensor(hwMap, intakeSensorName);
        breakBeam = hwMap.get(DigitalChannel.class, breakBeamIntakeName);
        breakBeam.setMode(DigitalChannel.Mode.INPUT);
        sorter.setSlot(slot);
    }

    public void setState(States state){
        this.state = state;
    }

    public void setAction(Actions action){
        this.currentAction = action;
        switch (action) {
            case FindGreen: {
                if (greenArtifactNumber > 0) {
                    if (currentColor != Green) {
                        if (sorter.getColor(sorter.getNextSlot()) == Green) {
                            setAction(Actions.NextSlot);
                        } else {
                            setAction(Actions.PreviousSlot);
                        }
                    }
                }
            }
            break;
            case FindPurple: {
                if (purpleArtifactNumber > 0) {
                    if (currentColor != Purple) {
                        if (sorter.getColor(sorter.getNextSlot()) == Purple) {
                            setAction(Actions.NextSlot);
                        } else {
                            setAction(Actions.PreviousSlot);
                        }
                    }
                }
            }
            case NextSlot: {
                sorter.setNextSlot();
                setIntakeForSwitch();
            }
            break;
            case PreviousSlot: {
                sorter.setPreviousSlot();
                setIntakeForSwitch();
            }
            break;
        }
    }

    public void setPower(IntakeMotor.States state){
        lastPower = state;
        intakeMotor.setState(state);
    }

    public IntakeMotor.States getPower(){
        return lastPower;
    }

    public void setCollectType(CollectTypes CollectTypes) {
        this.collectType = CollectTypes;
    }

    public CollectTypes getCollectType() {
        return collectType;
    }

    public States getState(){
        return state;
    }

    public void setIntakeForSwitch(){
        canStop = false;
        helpingSorter = true;
        setPower(IntakeMotor.States.Collect);
        intakeHelper.reset();
    }

    public void findColor(Constants.Color color){
        switch (color){
            case Purple:
                setAction(Actions.FindPurple);
                break;
            case Green:
                setAction(Actions.FindGreen);
                break;
        }
    }

    public boolean atTarget(){
        return sorter.atTarget();
    }

    public void updateColor() {
        if (intakeMotor.getState() != IntakeMotor.States.Wait) {
            if (intakeMotor.getPower() >= 0) {
                breakBeamCollected = breakBeam.getState();
                boolean artifactSeen = sensor.isInReach(0.7);
                if (breakBeamCollected || artifactSeen) {
                    artifactPassThrough = true;
                    timer.reset();
                }
                if (artifactPassThrough) {
                    sensor.update();
                    if (sensor.getColor() != None && currentColor == None && !colorAssignedToCurrentSample) {
                        Constants.Color color = sensor.getColor();
                        sorter.setColor(color, currentSlot);
                        colorAssignedToCurrentSample = true;
                    }
                }
            } else if (intakeMotor.getPower() <= 0) {
                breakBeamCollected = breakBeam.getState();
                if (breakBeamCollected) {
                    sorter.setColor(None, currentSlot);
                }
            }

        } else {
            breakBeamCollected = false;
        }

        if (timer.milliseconds() >= 400) {
            artifactPassThrough = false;
            colorAssignedToCurrentSample = false;
        }
    }

    @Override
    public void update() {
        sorter.update();
        if (helpingSorter && intakeHelper.milliseconds() >= 400) {
            setPower(IntakeMotor.States.Wait);
            helpingSorter = false;
            canStop = true;
        }

        currentSlot = sorter.getSlot();
        currentSlotNumber = sorter.getSlotNumber();
        currentColor = sorter.getColor(currentSlot);
        artifactNumber = 0;
        purpleArtifactNumber = 0;
        greenArtifactNumber = 0;

        switch(sorterColors[0]){
            case Purple:
                purpleArtifactNumber++;
                artifactNumber++;
                break;
            case Green:
                greenArtifactNumber++;
                artifactNumber++;
                break;
        }
        switch(sorterColors[1]){
            case Purple:
                purpleArtifactNumber++;
                artifactNumber++;
                break;
            case Green:
                greenArtifactNumber++;
                artifactNumber++;
                break;
        }
        switch(sorterColors[2]){
            case Purple:
                purpleArtifactNumber++;
                artifactNumber++;
                break;
            case Green:
                greenArtifactNumber++;
                artifactNumber++;
                break;
        }

        intakeMotor.update();
    }
}