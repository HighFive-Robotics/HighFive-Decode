package org.firstinspires.ftc.teamcode.Core.Module.Others;

import static org.firstinspires.ftc.teamcode.Constants.DeviceNames.liftServoLB;
import static org.firstinspires.ftc.teamcode.Constants.DeviceNames.liftServoRB;
import static org.firstinspires.ftc.teamcode.Constants.DeviceNames.liftServoRF;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Core.Hardware.HighModule;
import org.firstinspires.ftc.teamcode.Core.Hardware.HighServo;

public class Lift extends HighModule {

    HighServo servoRB, servoRF, servoLB;

    enum Actions{
        Raise,
        Stop
    }

    public Lift(HardwareMap hwMap){
        servoLB = HighServo.Builder.startBuilding()
                .setServo(hwMap.get(CRServo.class, liftServoLB))
                .setContinousRotationRunMode()
                .build();
        servoRF = HighServo.Builder.startBuilding()
                .setServo(hwMap.get(CRServo.class, liftServoRF))
                .setContinousRotationRunMode()
                .build();
        servoRB = HighServo.Builder.startBuilding()
                .setServo(hwMap.get(CRServo.class, liftServoRB))
                .setContinousRotationRunMode()
                .build();
    }

    public void setAction(Actions action){
        switch (action){
            case Raise:
                break;
            case Stop:
                setPower(0);
                break;
        }
    }

    public void setPower(double power){
        servoLB.setPower(-power);
        servoRF.setPower(power);
        servoRB.setPower(power);
    }

    @Override
    public void update() {
        servoRB.update();
        servoRF.update();
        servoLB.update();
    }
}
