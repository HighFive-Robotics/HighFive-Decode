package org.firstinspires.ftc.teamcode.Core.Module.Others;

import static org.firstinspires.ftc.teamcode.Constants.DeviceNames.blockerServoName;
import static org.firstinspires.ftc.teamcode.Constants.DeviceNames.brakeName;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Core.Hardware.HighModule;
import org.firstinspires.ftc.teamcode.Core.Hardware.HighServo;

@Config
public class BrakePiston extends HighModule {
    public HighServo servo;
    public static double BrakingPosition = 1;
    public static double FloatingPosition = 0;

    public enum States {
        Brake,
        Float
    }

    public States state;

    public BrakePiston(HardwareMap hwMap, double initPosition, boolean isAuto) {
        servo = HighServo.Builder.startBuilding(  )
                .setServo(hwMap.get(Servo.class, brakeName))
                .setStandardRunMode()
                .setInitPosition(initPosition, isAuto)
                .build();
    }

    public void setState(States state) {
        this.state = state;
        switch (state) {
            case Brake:
                setTarget(BrakingPosition);
                break;
            case Float:
                setTarget(FloatingPosition);
                break;
        }
    }

    public void setState(BrakePiston.States state, double time) {
        this.state = state;
        switch (state) {
            case Brake:
                setTarget(BrakingPosition, time);
                break;
            case Float:
                setTarget(FloatingPosition, time);
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

    @Override
    public boolean atTarget() {
        return servo.atTarget();
    }

    public double getTarget() {
        return target;
    }

    @Override
    public void update() {
        servo.update();
    }
}