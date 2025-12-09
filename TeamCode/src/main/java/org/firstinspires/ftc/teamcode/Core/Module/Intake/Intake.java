package org.firstinspires.ftc.teamcode.Core.Module.Intake;

import static org.firstinspires.ftc.teamcode.Constants.DeviceNames.breakBeamIntakeName;
import static org.firstinspires.ftc.teamcode.Constants.DeviceNames.intakeSensorName;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Core.Hardware.HighModule;
import org.firstinspires.ftc.teamcode.Core.Hardware.HighSensor;


public class Intake extends HighModule {
    public IntakeMotor intakeMotor;
    public Sorter sorter;
    public HighSensor sensor;
    public DigitalChannel breakBeam;

    IntakeActions action = IntakeActions.Collect;

    public enum IntakeActions{
        Collect,
        Transfer,
        Wait,
        Park
    }
    public Intake(HardwareMap hwMap){
        intakeMotor = new IntakeMotor(hwMap);
        sorter = new Sorter(hwMap);
        sensor = new HighSensor(hwMap, intakeSensorName);
        breakBeam = hwMap.get(DigitalChannel.class, breakBeamIntakeName);
    }
    public Intake(HardwareMap hwMap, Sorter.Slots slot){
        intakeMotor = new IntakeMotor(hwMap);
        sorter = new Sorter(hwMap);
        sensor = new HighSensor(hwMap, intakeSensorName);
        breakBeam = hwMap.get(DigitalChannel.class, breakBeamIntakeName);
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

    @Override
    public void update() {
        if(sorter.getState() == Sorter.States.Automated){
            if(action == IntakeActions.Collect && !sorter.isFull){
                if(sorter.getColor(sorter.getSlot()) != Constants.Color.None){
                    sorter.setNextSlot();
                }
            }
        }
        intakeMotor.update();
        sorter.update();
    }
}
