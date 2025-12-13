package org.firstinspires.ftc.teamcode.Core.Module.Intake;

import static org.firstinspires.ftc.teamcode.Constants.Color.Green;
import static org.firstinspires.ftc.teamcode.Constants.Color.None;
import static org.firstinspires.ftc.teamcode.Constants.Color.Purple;
import static org.firstinspires.ftc.teamcode.Constants.DeviceNames.breakBeamIntakeName;
import static org.firstinspires.ftc.teamcode.Constants.DeviceNames.intakeSensorName;
import static org.firstinspires.ftc.teamcode.Constants.Intake.SorterConstants.artifactNumber;
import static org.firstinspires.ftc.teamcode.Constants.Intake.SorterConstants.greenArtifactNumber;
import static org.firstinspires.ftc.teamcode.Constants.Intake.SorterConstants.purpleArtifactNumber;

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

    Sorter.Slots currentSlot;
    Constants.Color currentColor;
    int currentSlotNumber;

    ElapsedTime timer = new ElapsedTime();

    public boolean artifactPassThrough = false;
    public boolean breakBeamCollected = false;
    private boolean colorAssignedToCurrentSample = false;

    IntakeActions action = IntakeActions.Collect;
    CollectTypes collectType = CollectTypes.Sorted;

    public enum IntakeActions{
        Collect,
        FindGreen,
        FindPurple,
        Wait,
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

    public void setAction(IntakeActions action){
        this.action = action;
        switch (action){
           case FindGreen:{
               if(greenArtifactNumber > 0){
                   if(currentColor != Green){
                       if(sorter.getColor(sorter.getNextSlot()) == Green){
                            sorter.setNextSlot();
                       } else {
                           sorter.setPreviousSlot();
                       }
                   }
               }
           }
           break;
            case FindPurple:{
                if(purpleArtifactNumber > 0){
                    if(currentColor != Purple){
                        if(sorter.getColor(sorter.getNextSlot()) == Purple){
                            sorter.setNextSlot();
                        } else {
                            sorter.setPreviousSlot();
                        }
                    }
                }
            }
            break;
        }
    }

    public void setPower(IntakeMotor.States state){
        intakeMotor.setState(state);
    }

    public void setCollectType(CollectTypes CollectTypes) {
        this.collectType = CollectTypes;
    }

    public CollectTypes getCollectType() {
        return collectType;
    }

    public IntakeActions getLastAction(){
        return action;
    }

    public void updateColor(){
        if(intakeMotor.getState() != IntakeMotor.States.Wait){
            if(intakeMotor.getPower() >= 0){
                breakBeamCollected = breakBeam.getState();
                if(breakBeamCollected){
                    artifactPassThrough = true;
                    timer.reset();
                }
                if(artifactPassThrough){
                    sensor.update();
                    if(sensor.getColor() != None && currentColor == None && !colorAssignedToCurrentSample){
                        Constants.Color color = sensor.getColor();
                        sorter.setColor(color, currentSlot);
                        colorAssignedToCurrentSample = true;
                        artifactNumber++;
                        switch(color){
                            case Purple:
                                purpleArtifactNumber++;
                                break;
                            case Green:
                                greenArtifactNumber++;
                                break;
                        }
                    }
                }
            } else if(intakeMotor.getPower() <= 0){
                breakBeamCollected = breakBeam.getState();
                if(breakBeamCollected){
                    sorter.setColor(None, currentSlot);
                }
            }

        } else {
            breakBeamCollected = false;
        }

        if(timer.milliseconds() >= 250){
            artifactPassThrough = false;
            colorAssignedToCurrentSample = false;
        }
    }

    @Override
    public void update() {
        currentSlot = sorter.getSlot();
        currentSlotNumber = sorter.getSlotNumber();
        currentColor = sorter.getColor(currentSlot);
        switch (collectType){
            case Sorted:
                if(sorter.getState() == Sorter.States.Automated){
                    if(!sorter.isFull){
                        if(currentColor != None && !artifactPassThrough){
                            sorter.setNextSlot();
                        }
                    }
                    updateColor();
                }
                break;
            case Mix:
                if(sorter.getState() == Sorter.States.Automated && artifactNumber < 1){
                    if(!sorter.isFull){
                        if(currentColor != None && !artifactPassThrough){
                            sorter.setNextSlot();
                        }
                    }
                    updateColor();
                }
                break;
            case Normal:
                break;
        }
        intakeMotor.update();
        sorter.update();
    }
}