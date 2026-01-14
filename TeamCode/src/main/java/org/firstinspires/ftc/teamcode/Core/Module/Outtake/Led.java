package org.firstinspires.ftc.teamcode.Core.Module.Outtake;

import static org.firstinspires.ftc.teamcode.Constants.DeviceNames.blockerServoName;
import static org.firstinspires.ftc.teamcode.Constants.DeviceNames.ledName;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Core.Hardware.HighModule;

import org.firstinspires.ftc.teamcode.Core.Hardware.HighServo;

public class Led extends HighModule {

    HighServo led;

    public Led(HardwareMap hw){
        led =   HighServo.Builder.startBuilding()
                .setServo(hw.get(Servo.class , ledName))
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
            case None:
                setTarget(0);
                break;
        }
    }

    public void setTarget(double target) {
        this.target = target;
        led.setPosition(target);
    }

    public void setTarget(double target, double time) {
        this.target = target;
        led.setPosition(target, time);
    }

    @Override
    public boolean atTarget(){
        return led.atTarget();
    }

    public double getTarget() {
        return target;
    }

    @Override
    public void update() {
        led.update();
    }
}
