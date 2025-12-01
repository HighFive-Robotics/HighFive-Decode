package org.firstinspires.ftc.teamcode.Core.Module.Intake;

import static org.firstinspires.ftc.teamcode.Constants.Intake.SorterConstants.Position;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Core.Hardware.HighModule;


public class Intake extends HighModule {
    public IntakeMotor intakeMotor;
    public Joint joint;
    public Sorter sorter;

    IntakeActions action;

    public enum IntakeActions{
        Collect,
        CollectLowPower,
        Spit,
        Transfer,
        Wait,
        Park
    }
    public Intake(HardwareMap hwMap){
        intakeMotor = new IntakeMotor(hwMap);
        joint = new Joint(hwMap);
        sorter = new Sorter(hwMap, Position, false);
    }
    public Intake(HardwareMap hwMap ,double initPosition, boolean isAuto){
        intakeMotor = new IntakeMotor(hwMap);
        joint = new Joint(hwMap, initPosition, isAuto);
        sorter = new Sorter(hwMap, Position, true);
    }
    public void setAction(IntakeActions action){
        this.action = action;
        switch (action){
            case Collect:
                intakeMotor.setState(IntakeMotor.States.Collect);
                break;
            case Wait:
                intakeMotor.setState(IntakeMotor.States.Wait);
                break;
            case Spit:
                intakeMotor.setState(IntakeMotor.States.Spit);
                break;
            case Park:
                intakeMotor.setState(IntakeMotor.States.Wait);
                intakeMotor.disable();
                joint.setState(Joint.States.Park);
                break;
            case CollectLowPower:
                intakeMotor.setPower(0.7);
                break;
        }
    }

    public IntakeActions getLastAction(){
        return action;
    }

    @Override
    public void update() {
        intakeMotor.update();
        joint.update();
        sorter.update();
    }
}
