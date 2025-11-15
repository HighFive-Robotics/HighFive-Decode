package org.firstinspires.ftc.teamcode.Core.Module.Outtake;

import static org.firstinspires.ftc.teamcode.Constants.DeviceNames.blockerServoName;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Core.Hardware.HighModule;
import org.firstinspires.ftc.teamcode.Core.Hardware.HighServo;

@Config
public class BlockerOuttake extends HighModule {
    HighServo servo;
    public static double ClosedPosition = 0.1;
    public static double OpenPosition = 0.6;

    public enum States {
        Open,
        Close
    }

    States state;

    public BlockerOuttake(HardwareMap hwMap, double initPosition, boolean isAuto) {
        servo = HighServo.Builder.startBuilding()
                .setServo(hwMap.get(Servo.class , blockerServoName))
                .setStandardRunMode()
                .setInitPosition(initPosition , isAuto)
                .build();
    }

    public void setState(BlockerOuttake.States state){
        this.state = state;
        switch (state){
            case Open:
                setTarget(OpenPosition);
                break;
            case Close:
                setTarget(ClosedPosition);
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

    @Override
    public void update() {
        servo.update();
    }
}
