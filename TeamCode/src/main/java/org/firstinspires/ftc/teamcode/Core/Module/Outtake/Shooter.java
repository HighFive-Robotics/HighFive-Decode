package org.firstinspires.ftc.teamcode.Core.Module.Outtake;

import static org.firstinspires.ftc.teamcode.Constants.DeviceNames.shooterMotorDownName;
import static org.firstinspires.ftc.teamcode.Constants.DeviceNames.shooterMotorUpName;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Core.Hardware.HighModule;
import org.firstinspires.ftc.teamcode.Core.Hardware.HighMotor;

public class Shooter extends HighModule {
    public HighMotor motorUp , motorDown;
    public double velocity;
    public static double encoderResolution  ,kp = 0.001, kd = 0,ki = 0.002,kf = 0.00016;

    public Shooter(HardwareMap hwMap){
        motorDown = HighMotor.Builder.startBuilding()
                .setMotor(hwMap.get(DcMotorEx.class, shooterMotorDownName))
                .setRunMode(HighMotor.RunMode.Velocity)
                .setReverseMotor(false)
                .setEncoder(true , false)
                .setEncoderResolution(28)
                .setWheelDiameter(0.072)
                .setVelocityPIDCoefficients(kp,ki,kd,kf,1)
                .build();
        motorUp = HighMotor.Builder.startBuilding()
                .setMotor(hwMap.get(DcMotorEx.class, shooterMotorUpName))
                .setRunMode(HighMotor.RunMode.Standard)
                .setReverseMotor(false)
                .build();
    }

    public void setTargetVelocity(double velocity){
        motorDown.setTarget(velocity);
    }

    @Override
    public void update() {

        motorUp.update();
        //motorUp.setTarget(motorUp.getPower());
        motorUp.setPower(motorDown.getPower());
        motorDown.update();
    }
}
