package org.firstinspires.ftc.teamcode.Core.Module.Outtake;

import static org.firstinspires.ftc.teamcode.Constants.DeviceNames.blockerServoName;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Core.Hardware.HighModule;
import org.firstinspires.ftc.teamcode.Core.Hardware.HighServo;

@Config
public class Blocker extends HighModule {
    public HighServo servo;
    public static double ClosedPosition = 0.58;
    public static double OpenPosition = 0.85;

    public enum States {
        Open,
        Close
    }

    public States state;

    public Blocker(HardwareMap hwMap, double initPosition, boolean isAuto) {
        servo = HighServo.Builder.startBuilding()
                .setServo(hwMap.get(Servo.class, blockerServoName))
                .setStandardRunMode()
                .setInitPosition(initPosition, isAuto)
                .build();
    }

    public void setState(States state) {
        this.state = state;
        switch (state) {
            case Open:
                setTarget(OpenPosition);
                break;
            case Close:
                setTarget(ClosedPosition);
                break;
        }
    }

    public void setState(Blocker.States state, double time) {
        this.state = state;
        switch (state) {
            case Open:
                setTarget(OpenPosition, time);
                break;
            case Close:
                setTarget(ClosedPosition, time);
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