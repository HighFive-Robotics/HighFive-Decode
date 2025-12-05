package org.firstinspires.ftc.teamcode.Core.Module.Intake;

import static org.firstinspires.ftc.teamcode.Constants.DeviceNames.shooterMotorDownName;
import static org.firstinspires.ftc.teamcode.Constants.DeviceNames.sorterServoName;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Core.Hardware.HighEncoder;
import org.firstinspires.ftc.teamcode.Core.Hardware.HighModule;
import org.firstinspires.ftc.teamcode.Core.Hardware.HighServo;

@Config
public class Sorter extends HighModule {
    HighServo servo;
    HighEncoder encoder;
    public Sorter(HardwareMap hwMap, double initPosition, boolean isAuto) {
        servo = HighServo.Builder.startBuilding()
                .setServo(hwMap.get(CRServo.class , sorterServoName))
                .setContinousRotationRunMode()
                .build();
        encoder = new HighEncoder(hwMap.get(DcMotorEx.class, shooterMotorDownName), 0, false);
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
