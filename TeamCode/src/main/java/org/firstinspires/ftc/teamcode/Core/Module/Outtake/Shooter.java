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
    public BlockerOuttake blocker;
    public double velocity;
    public static double kp = 0.0007, kd = 5e-8,ki = 0.005,kf = 0.00022;
    public static double encoderResolution=28;

    public Shooter(HardwareMap hwMap){
        motorUp = HighMotor.Builder.startBuilding()
                .setMotor(hwMap.get(DcMotorEx.class, shooterMotorUpName))
                .setRunMode(HighMotor.RunMode.Velocity)
                .setReverseMotor(false)
                .setEncoder(true , true)
                .setEncoderResolution(28)
                .setMotorRPM(HighMotor.MotorRPM.RPM6000)
                .setWheelDiameter(0.072)
                .setVelocityPIDCoefficients(kp,ki,kd,kf)
                .setUseZeroPowerBehaviour(false)
                .build();
        motorUp.setTolerance(0.1);
        motorDown = HighMotor.Builder.startBuilding()
                .setMotor(hwMap.get(DcMotorEx.class, shooterMotorDownName))
                .setRunMode(HighMotor.RunMode.Standard)
                .setReverseMotor(true)
                .setUseZeroPowerBehaviour(false)
                .build();
        blocker = new BlockerOuttake(hwMap,blocker.ClosedPosition,true);
    }

    public void setTargetVelocity(double velocity){
        motorUp.setTarget(velocity);
    }

    @Override
    public boolean atTarget(){
        return Math.abs(motorUp.getTarget() - motorUp.getCurrentVelocity()) <= motorUp.getTolerance();
    }
    public double getVelocityError(){
        return Math.abs(motorUp.getTarget() - motorUp.getCurrentVelocity());
    }
    @Override
    public void update() {

        motorUp.update();
        motorUp.setVelocityPIDCoefficients(kp,ki,kd,kf);
        //motorUp.setTarget(motorUp.getPower());
        motorDown.setPower(motorUp.getPower());
        motorDown.update();
        blocker.update();
    }
}
