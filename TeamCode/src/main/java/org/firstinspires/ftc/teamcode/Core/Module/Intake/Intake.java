package org.firstinspires.ftc.teamcode.Core.Module.Intake;

import static org.firstinspires.ftc.teamcode.Constants.Color.None;
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

    ElapsedTime timer = new ElapsedTime();

    public boolean artifactPassThrough = false;
    public boolean breakBeamCollected = false;

    IntakeActions action = IntakeActions.Collect;
    CollectTypes collectType = CollectTypes.Normal;

    public enum IntakeActions{
        Collect,
        Wait,
    }

    public enum CollectTypes {
        Sorted,
        Normal,
        Mix
    }

    public Intake(HardwareMap hwMap){
        intakeMotor = new IntakeMotor(hwMap);
        sorter = new Sorter(hwMap,0);
        sensor = new HighSensor(hwMap, intakeSensorName);
        breakBeam = hwMap.get(DigitalChannel.class, breakBeamIntakeName);
        breakBeam.setMode(DigitalChannel.Mode.INPUT);
    }
    public Intake(HardwareMap hwMap, Sorter.Slots slot){
        intakeMotor = new IntakeMotor(hwMap);
        sorter = new Sorter(hwMap,0);
        sensor = new HighSensor(hwMap, intakeSensorName);
        breakBeam = hwMap.get(DigitalChannel.class, breakBeamIntakeName);
        breakBeam.setMode(DigitalChannel.Mode.INPUT);
        sorter.setSlot(slot);
    }
    public void setAction(IntakeActions action){
        this.action = action;
        switch (action){
        }
    }

    public void setPower(IntakeMotor.States state){
        intakeMotor.setState(state);
    }

    public IntakeActions getLastAction(){
        return action;
    }

    public void updateColor(){
        breakBeamCollected = breakBeam.getState();
        if(breakBeamCollected){
            artifactPassThrough = true;
            timer.reset();
        }
        if(artifactPassThrough){
            sensor.update();
            if(sensor.getColor() != None && sorter.getColor(sorter.getSlot()) != None){
                Constants.Color color = sensor.getColor();
                sorter.setColor(sensor.getColor(), sorter.getSlot());
                artifactNumber++;
                switch(color){
                    case Purple:{
                        purpleArtifactNumber++;
                    }
                    break;
                    case Green:{
                        greenArtifactNumber++;
                    }
                    break;
                }
            }
        }

        if(timer.milliseconds() >= 750){
            artifactPassThrough = false;
        }
    }

    @Override
    public void update() {
        switch (collectType){
            case Sorted:{
                if(sorter.getState() == Sorter.States.Automated){
                    if(action == IntakeActions.Collect && !sorter.isFull){
                        if(sorter.getColor(sorter.getSlot()) != None){
                            sorter.setNextSlot();
                        }
                    }
                    updateColor();
                }
            }
            break;
            case Mix:{
                if(sorter.getState() == Sorter.States.Automated && artifactNumber < 1){
                    if(action == IntakeActions.Collect && !sorter.isFull){
                        if(sorter.getColor(sorter.getSlot()) != None){
                            sorter.setNextSlot();
                        }
                    }
                    updateColor();
                }
            }
            break;
            case Normal:
            break;
        }
        intakeMotor.update();
        sorter.update();
    }
}
