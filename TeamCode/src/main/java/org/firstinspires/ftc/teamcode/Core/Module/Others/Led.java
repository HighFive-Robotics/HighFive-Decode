package org.firstinspires.ftc.teamcode.Core.Module.Others;

import static org.firstinspires.ftc.teamcode.Constants.DeviceNames.ledName1;
import static org.firstinspires.ftc.teamcode.Constants.DeviceNames.ledName2;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Core.Hardware.HighModule;

import org.firstinspires.ftc.teamcode.Core.Hardware.HighServo;

public class Led extends HighModule {

    HighServo led1, led2;

    public Led(HardwareMap hw){
        led1=   HighServo.Builder.startBuilding()
                .setServo(hw.get(Servo.class , ledName1))
                .setStandardRunMode()
                .setInitPosition(0,true)
                .build();
        led2 =   HighServo.Builder.startBuilding()
                .setServo(hw.get(Servo.class , ledName2))
                .setStandardRunMode()
                .setInitPosition(0,true)
                .build();
    }

    public void setColor(Constants.Color color) {
        switch (color){
            case Red:
                setTarget(0.28);
                break;
            case Green:
                setTarget(0.5);
                break;
            case Purple:
                setTarget(0.722);
                break;
            case Yellow:
                setTarget(0.345);
                break;
            case None:
                setTarget(0);
                break;
        }
    }

    public void setTarget(double target) {
        this.target = target;
        led1.setPosition(target);
        led2.setPosition(target);
    }

    public void setTarget(double target, double time) {
        this.target = target;
        led1.setPosition(target, time);
        led2.setPosition(target);
    }

    @Override
    public boolean atTarget(){
        return led1.atTarget();
    }

    public double getTarget() {
        return target;
    }

    @Override
    public void update() {
        led1.update();
        led2.update();
    }
}
