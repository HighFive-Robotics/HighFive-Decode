package org.firstinspires.ftc.teamcode.Core.Module.Outtake;

import static org.firstinspires.ftc.teamcode.Constants.DeviceNames.shooterMotorDownName;
import static org.firstinspires.ftc.teamcode.Constants.DeviceNames.shooterMotorUpName;
import static org.firstinspires.ftc.teamcode.Constants.ShooterConstants.ShooterBackWheelParams.encoderResolutionBack;
import static org.firstinspires.ftc.teamcode.Constants.ShooterConstants.ShooterBackWheelParams.kaBack;
import static org.firstinspires.ftc.teamcode.Constants.ShooterConstants.ShooterBackWheelParams.kdBack;
import static org.firstinspires.ftc.teamcode.Constants.ShooterConstants.ShooterBackWheelParams.kfBack;
import static org.firstinspires.ftc.teamcode.Constants.ShooterConstants.ShooterBackWheelParams.kiBack;
import static org.firstinspires.ftc.teamcode.Constants.ShooterConstants.ShooterBackWheelParams.kpBack;
import static org.firstinspires.ftc.teamcode.Constants.ShooterConstants.ShooterBackWheelParams.ksBack;
import static org.firstinspires.ftc.teamcode.Constants.ShooterConstants.ShooterBackWheelParams.wheelDiameterBack;
import static org.firstinspires.ftc.teamcode.Constants.ShooterConstants.ShooterFlyWheelParams.encoderResolutionFly;
import static org.firstinspires.ftc.teamcode.Constants.ShooterConstants.ShooterFlyWheelParams.kaFly;
import static org.firstinspires.ftc.teamcode.Constants.ShooterConstants.ShooterFlyWheelParams.kdFly;
import static org.firstinspires.ftc.teamcode.Constants.ShooterConstants.ShooterFlyWheelParams.kfFly;
import static org.firstinspires.ftc.teamcode.Constants.ShooterConstants.ShooterFlyWheelParams.kiFly;
import static org.firstinspires.ftc.teamcode.Constants.ShooterConstants.ShooterFlyWheelParams.kpFly;
import static org.firstinspires.ftc.teamcode.Constants.ShooterConstants.ShooterFlyWheelParams.ksFly;
import static org.firstinspires.ftc.teamcode.Constants.ShooterConstants.ShooterFlyWheelParams.wheelDiameterFly;


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
    public double velocityUp,velocityDown, tolerance;
    public double targetUp , targetDown;
    public Shooter(HardwareMap hwMap){
        motorDown = HighMotor.Builder.startBuilding()
                .setMotor(hwMap.get(DcMotorEx.class, shooterMotorDownName))
                .setRunMode(HighMotor.RunMode.Velocity)
                .setReverseMotor(false)
                .setEncoder(true , false)
                .setEncoderResolution(encoderResolutionFly)
                .setMotorRPM(HighMotor.MotorRPM.RPM6000)
                .setWheelDiameter(wheelDiameterFly)
                .useVoltageComensationForVelocity(true)
                .setVelocityPIDCoefficients(kpFly,kiFly,kdFly,kfFly,ksFly,kaFly,1)
                .setUseZeroPowerBehaviour(false)
                .build();
        motorDown.setTolerance(0.075);
        motorUp = HighMotor.Builder.startBuilding()
                .setMotor(hwMap.get(DcMotorEx.class, shooterMotorUpName))
                .setRunMode(HighMotor.RunMode.Velocity)
                .setReverseMotor(false)
                .setEncoder(true , false)
                .setEncoderResolution(encoderResolutionBack)
                .setMotorRPM(HighMotor.MotorRPM.RPM6000)
                .setWheelDiameter(wheelDiameterBack)
                .useVoltageComensationForVelocity(true)
                .setVelocityPIDCoefficients(kpBack,kiBack,kdBack,kfBack,ksBack,kaBack,1)
                .setUseZeroPowerBehaviour(false)
                .build();
        motorUp.setTolerance(0.075);
        blocker = new Blocker(hwMap, Blocker.OpenPosition,true);
        tolerance = motorUp.getTolerance();
    }
    public void setPIDCoefDown(double kp , double kd , double ki , double kf){
        motorDown.setVelocityPIDCoefficients(kp,ki,kd,kf,1);
    }
    public void setPIDCoefUp(double kp , double kd , double ki , double kf){
        motorUp.setVelocityPIDCoefficients(kp,ki,kd,kf,1);
    }
    public void updateCoefDown(){
        motorDown.setVelocityPIDFSA(kpFly,kiFly,kdFly,kfFly,ksFly,kaFly,1);
    }
    public void updateCoefUp(){
        motorUp.setVelocityPIDFSA(kpBack,kiBack,kdBack,kfBack,ksBack,kaBack,1);
    }
    public void updateAllCoef(){
        updateCoefDown();
        updateCoefUp();
    }
    public void nanUp(){
        motorUp.setVelocityPIDFSA(0,0,0,0,0,0,1);
    }
    public void nanDown(){
        motorDown.setVelocityPIDFSA(0,0,0,0,0,0,1);
    }


    public void setFullTargetVelocity(double velocity){
        double veloUp = Range.scale(velocity,-7.2 , 7.2 , -0.7 , 0.7);
        this.targetUp = veloUp;
        this.targetDown = velocity;
        motorUp.setTarget(veloUp);
        motorDown.setTarget(velocity);
    }
    public void setUpTargetVelocity(double velocity){
        this.targetUp = velocity;
        motorUp.setTarget(velocity);
    }
    public void setDownTargetVelocity(double velocity){
        this.targetDown = velocity;
        motorDown.setTarget(velocity);
    }
    public void setTargetVelocity(double down , double up){
        setUpTargetVelocity(up);
        setUpTargetVelocity(down);
    }
    public boolean upAtTarget(){
        return Math.abs(targetUp-velocityUp) <= tolerance;
    }
    public boolean downAtTarget(){
        return Math.abs(targetDown-velocityDown) <= tolerance;
    }
    @Override
    public boolean atTarget(){
        return upAtTarget() && downAtTarget();
    }
    public double getVelocityErrorUp(){
        return Math.abs(targetUp - velocityUp);
    }
    public double getVelocityErrorDown(){
        return Math.abs(targetUp - velocityDown);
    }
//    public double getVelociyFromDistance(double distance){
//        return Range.clip( 8.9416e-10 * distance* distance* distance* distance -7.58832e-7 * distance* distance* distance + 0.000234922 * distance * distance -0.0238624 * distance + 4.0805,3,5.35);
//    }
    @Override
    public double getTarget(){
        return target;
    }
    @Override
    public void update() {
        velocityUp = motorUp.getCurrentVelocity();
        velocityDown = motorDown.getCurrentVelocity();
        motorUp.update();
        motorDown.update();
        //motorUp.setVelocityPIDFSA(kp,ki,kd,kf,ks,ka,1);
        blocker.update();
    }
}