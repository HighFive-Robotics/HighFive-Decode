package org.firstinspires.ftc.teamcode.Core.Module.Outtake;

import static org.firstinspires.ftc.teamcode.Constants.DeviceNames.shooterMotorDownName;
import static org.firstinspires.ftc.teamcode.Constants.DeviceNames.shooterMotorUpName;
import static org.firstinspires.ftc.teamcode.Constants.ShooterConstants.ka;
import static org.firstinspires.ftc.teamcode.Constants.ShooterConstants.kd;
import static org.firstinspires.ftc.teamcode.Constants.ShooterConstants.kf;
import static org.firstinspires.ftc.teamcode.Constants.ShooterConstants.ki;
import static org.firstinspires.ftc.teamcode.Constants.ShooterConstants.kp;
import static org.firstinspires.ftc.teamcode.Constants.ShooterConstants.encoderResolution;
import static org.firstinspires.ftc.teamcode.Constants.ShooterConstants.ks;
import static org.firstinspires.ftc.teamcode.Constants.ShooterConstants.wheelDiameter;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Core.Hardware.HighModule;
import org.firstinspires.ftc.teamcode.Core.Hardware.HighMotor;

@Config
public class Shooter extends HighModule {
    public HighMotor motorUp , motorDown;
    public BlockerOuttake blocker;
    public double velocity, tolerance;

    public Shooter(HardwareMap hwMap){
        motorUp = HighMotor.Builder.startBuilding()
                .setMotor(hwMap.get(DcMotorEx.class, shooterMotorUpName))
                .setRunMode(HighMotor.RunMode.Velocity)
                .setReverseMotor(false)
                .setEncoder(true , true)
                .setEncoderResolution(encoderResolution)
                .setMotorRPM(HighMotor.MotorRPM.RPM6000)
                .setWheelDiameter(wheelDiameter)
                .useVoltageComensationForVelocity(true)
                .setVelocityPIDCoefficients(kp,ki,kd,kf,ks,ka,1)
                .setUseZeroPowerBehaviour(false)
                .build();
        motorUp.setTolerance(0.1);
        motorDown = HighMotor.Builder.startBuilding()
                .setMotor(hwMap.get(DcMotorEx.class, shooterMotorDownName))
                .setRunMode(HighMotor.RunMode.Standard)
                .setReverseMotor(true)
                .setUseZeroPowerBehaviour(false)
                .build();
        blocker = new BlockerOuttake(hwMap, BlockerOuttake.ClosedPosition,true);
        tolerance = motorUp.getTolerance();
    }

    public void setTargetVelocity(double velocity){
        this.target = velocity;
        motorUp.setTarget(velocity);
    }

    @Override
    public boolean atTarget(){
        return Math.abs(target- velocity) <= tolerance;
    }
    public double getVelocityError(){
        return Math.abs(target - velocity);
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
