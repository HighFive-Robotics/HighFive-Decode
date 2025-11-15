package org.firstinspires.ftc.teamcode.Core.Module.Intake;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Core.Hardware.HighModule;


public class Intake extends HighModule {
    public MotorIntake motorIntake;
    public Joint joint;
    public Sorter sorter;

    public enum IntakeActions{
        Collect,
        Spit,
        Transfer,
        Wait,
        Park
    }
    public Intake(HardwareMap hwMap){
        motorIntake = new MotorIntake(hwMap);
        joint = new Joint(hwMap);
        sorter = new Sorter(hwMap, Sorter.Position, false);
    }
    public Intake(HardwareMap hwMap ,double initPosition, boolean isAuto){
        motorIntake = new MotorIntake(hwMap);
        joint = new Joint(hwMap, initPosition, isAuto);
        if(isAuto) {
            sorter = new Sorter(hwMap, Sorter.Position, true);
        }else {
            joint = new Joint(hwMap);
        }
    }
    public void setAction(IntakeActions action){
        switch (action){
            case Collect:
                motorIntake.setState(MotorIntake.States.Collect);
                break;
            case Wait:
                motorIntake.setState(MotorIntake.States.Wait);
                break;
            case Spit:
                motorIntake.setState(MotorIntake.States.Spit);
                break;
            case Park:
                motorIntake.setState(MotorIntake.States.Wait);
                motorIntake.disable();
                joint.setState(Joint.States.Park);
                break;
        }
    }
    @Override
    public void update() {
        motorIntake.update();
        joint.update();
        sorter.update();
    }
}
