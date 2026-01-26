package org.firstinspires.ftc.teamcode.Core.Module.Outtake;

import static org.firstinspires.ftc.teamcode.Constants.DeviceNames.shooterMotorDownName;
import static org.firstinspires.ftc.teamcode.Constants.DeviceNames.shooterMotorUpName;
import static org.firstinspires.ftc.teamcode.Constants.ShooterConstants.encoderResolutionBack;
import static org.firstinspires.ftc.teamcode.Constants.ShooterConstants.ka;
import static org.firstinspires.ftc.teamcode.Constants.ShooterConstants.kd;
import static org.firstinspires.ftc.teamcode.Constants.ShooterConstants.kf;
import static org.firstinspires.ftc.teamcode.Constants.ShooterConstants.ki;
import static org.firstinspires.ftc.teamcode.Constants.ShooterConstants.kp;
import static org.firstinspires.ftc.teamcode.Constants.ShooterConstants.encoderResolution;
import static org.firstinspires.ftc.teamcode.Constants.ShooterConstants.ks;
import static org.firstinspires.ftc.teamcode.Constants.ShooterConstants.wheelDiameter;
import static org.firstinspires.ftc.teamcode.Constants.ShooterConstants.wheelDiameterBack;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Core.Algorithms.ActionSystem.CoreAction;
import org.firstinspires.ftc.teamcode.Core.Hardware.HighModule;
import org.firstinspires.ftc.teamcode.Core.Hardware.HighMotor;

@Config
public class Shooter extends HighModule {
    public HighMotor motorUp , motorDown;
    public Blocker blocker;
    public double velocity, tolerance;
    public Shooter(HardwareMap hwMap){
        motorUp = HighMotor.Builder.startBuilding()
                .setMotor(hwMap.get(DcMotorEx.class, shooterMotorUpName))
                .setRunMode(HighMotor.RunMode.Velocity)
                .setReverseMotor(true)
                .setEncoder(true , true)
                .setEncoderResolution(encoderResolution)
                .setMotorRPM(HighMotor.MotorRPM.RPM6000)
                .setWheelDiameter(wheelDiameter)
                .useVoltageComensationForVelocity(true)
                .setVelocityPIDCoefficients(kp,ki,kd,kf,ks,ka,1)
                .setUseZeroPowerBehaviour(false)
                .build();
        motorUp.setTolerance(0.075);
        motorDown = HighMotor.Builder.startBuilding()
                .setMotor(hwMap.get(DcMotorEx.class, shooterMotorDownName))
                .setRunMode(HighMotor.RunMode.Velocity)
                .setReverseMotor(true)
                .setEncoder(true , true)
                .setEncoderResolution(encoderResolutionBack)
                .setMotorRPM(HighMotor.MotorRPM.RPM6000)
                .setWheelDiameter(wheelDiameterBack)
                .useVoltageComensationForVelocity(true)
                .setVelocityPIDCoefficients(kp,ki,kd,kf,ks,ka,1)
                .setUseZeroPowerBehaviour(false)
                .build();
        blocker = new Blocker(hwMap, Blocker.OpenPosition,true);
        tolerance = motorUp.getTolerance();
    }

    public void setFullTargetVelocity(double velocity){
        this.target = velocity;
        motorUp.setTarget(velocity);
        motorDown.setTarget(velocity);
    }
    @Override
    public boolean atTarget(){
        return Math.abs(target- velocity) <= tolerance;
    }
    public double getVelocityError(){
        return Math.abs(target - velocity);
    }

    public double getVelociyFromDistance(double distance){
        return Range.clip( 8.9416e-10 * distance* distance* distance* distance -7.58832e-7 * distance* distance* distance + 0.000234922 * distance * distance -0.0238624 * distance + 4.0805,3,5.35);
    }

    @Override
    public double getTarget(){
        return target;
    }
    @Override
    public void update() {
        velocity = motorUp.getCurrentVelocity();
        motorUp.update();
        //motorUp.setVelocityPIDFSA(kp,ki,kd,kf,ks,ka,1);
        motorDown.setPower(motorUp.getPower());
        motorDown.update();
        blocker.update();
    }
}