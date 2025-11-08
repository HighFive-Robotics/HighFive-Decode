package org.firstinspires.ftc.teamcode.Core.Module.Intake;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Core.Hardware.HighModule;
import org.firstinspires.ftc.teamcode.Core.Hardware.HighServo;

public class Joint extends HighModule {

    public static double targetBlock = 0, targetPass = 0, targetPark = 0;

    HighServo servo;
    States state;

    public enum States {
        Block,
        Pass,
        Park
    }
    public Joint(HardwareMap hwMap){
        servo = HighServo.Builder.startBuilding()
                .setServo(hwMap.get(Servo.class, "DADAD"))
                .setStandardRunMode()
                .build();
    }
    public Joint(HardwareMap hwMap, double initPosition, boolean isAuto){
        servo = HighServo.Builder.startBuilding()
                .setServo(hwMap.get(Servo.class, "DADAD"))
                .setStandardRunMode()
                .setInitPosition(initPosition, isAuto)
                .build();
    }

    public void setState(States state){
        this.state = state;
        switch (state){
            case Park:
                setTarget(targetPark);
                break;
            case Pass:
                setTarget(targetPass);
                break;
            case Block:
                setTarget(targetBlock);
                break;
        }
    }


    public void setTarget(double target) {
        this.target = target;
        servo.setPosition(target);
    }

    public void setTarget(double target, double time) {
        this.target = target;
        servo.setPosition(target, time);
    }

    public double getTarget() {
        return target;
    }

    public States getState() {
        return state;
    }

    @Override
    public void update() {
        servo.update();
    }
}
