package org.firstinspires.ftc.teamcode.Core.Module.Outtake;

import static org.firstinspires.ftc.teamcode.Constants.DeviceNames.shooterMotorDownName;
import static org.firstinspires.ftc.teamcode.Constants.DeviceNames.shooterMotorUpName;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Core.Hardware.HighModule;
import org.firstinspires.ftc.teamcode.Core.Hardware.HighMotor;

@Config
public class Shooter extends HighModule {
    public HighMotor motorUp , motorDown;
    public double velocity;
    public static double encoderResolution  ,kp = 0.002 , kd = 0.00002,ki = 0.002,kf = 0.00010;

    public Shooter(HardwareMap hwMap){
        motorUp = HighMotor.Builder.startBuilding()
                .setMotor(hwMap.get(DcMotorEx.class, shooterMotorUpName))
                .setRunMode(HighMotor.RunMode.Velocity)
                .setReverseMotor(false)
                .setEncoder(true , true)
                .setEncoderResolution(28)
                .setWheelDiameter(0.072)
                .setVelocityPIDCoefficients(kp,ki,kd,kf)
                .build();
        motorDown = HighMotor.Builder.startBuilding()
                .setMotor(hwMap.get(DcMotorEx.class, shooterMotorDownName))
                .setRunMode(HighMotor.RunMode.Standard)
                .setReverseMotor(true)
                .build();
        motorUp.setTolerance(40);
    }

    public void setTargetVelocity(double velocity){
        motorUp.setTarget(velocity);
    }

    @Override
    public void update() {

        motorUp.update();
        //motorUp.setTarget(motorUp.getPower());
        motorDown.setPower(motorUp.getPower());
        motorDown.update();
    }
}
