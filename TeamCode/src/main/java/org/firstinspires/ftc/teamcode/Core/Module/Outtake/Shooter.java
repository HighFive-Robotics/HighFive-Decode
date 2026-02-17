package org.firstinspires.ftc.teamcode.Core.Module.Outtake;

import static org.firstinspires.ftc.teamcode.Constants.DeviceNames.shooterMotorDownName;
import static org.firstinspires.ftc.teamcode.Constants.DeviceNames.shooterMotorUpName;
import static org.firstinspires.ftc.teamcode.Constants.OuttakeConstants.ShooterBackWheelParams.encoderResolutionBack;
import static org.firstinspires.ftc.teamcode.Constants.OuttakeConstants.ShooterBackWheelParams.kC;
import static org.firstinspires.ftc.teamcode.Constants.OuttakeConstants.ShooterBackWheelParams.kaBack;
import static org.firstinspires.ftc.teamcode.Constants.OuttakeConstants.ShooterBackWheelParams.kdBack;
import static org.firstinspires.ftc.teamcode.Constants.OuttakeConstants.ShooterBackWheelParams.kfBack;
import static org.firstinspires.ftc.teamcode.Constants.OuttakeConstants.ShooterBackWheelParams.kiBack;
import static org.firstinspires.ftc.teamcode.Constants.OuttakeConstants.ShooterBackWheelParams.kpBack;
import static org.firstinspires.ftc.teamcode.Constants.OuttakeConstants.ShooterBackWheelParams.ksBack;
import static org.firstinspires.ftc.teamcode.Constants.OuttakeConstants.ShooterBackWheelParams.wheelDiameterBack;
import static org.firstinspires.ftc.teamcode.Constants.OuttakeConstants.ShooterFlyWheelParams.encoderResolutionFly;
import static org.firstinspires.ftc.teamcode.Constants.OuttakeConstants.ShooterFlyWheelParams.kaFly;
import static org.firstinspires.ftc.teamcode.Constants.OuttakeConstants.ShooterFlyWheelParams.kdFly;
import static org.firstinspires.ftc.teamcode.Constants.OuttakeConstants.ShooterFlyWheelParams.kfFly;
import static org.firstinspires.ftc.teamcode.Constants.OuttakeConstants.ShooterFlyWheelParams.kiFly;
import static org.firstinspires.ftc.teamcode.Constants.OuttakeConstants.ShooterFlyWheelParams.kpFly;
import static org.firstinspires.ftc.teamcode.Constants.OuttakeConstants.ShooterFlyWheelParams.ksFly;
import static org.firstinspires.ftc.teamcode.Constants.OuttakeConstants.ShooterFlyWheelParams.wheelDiameterFly;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Core.Hardware.HighModule;
import org.firstinspires.ftc.teamcode.Core.Hardware.HighMotor;

@Config
public class Shooter extends HighModule {
    public HighMotor motorUp, motorDown;
    public double velocityUp, velocityDown, upTolerance, downTolerance, toleranceCompensation = 0.2, upOffset = 0, downOffset = 0 , compOffset = 0;
    public double targetUp, targetDown;
    public boolean shouldCompensate = false;

    public Shooter(HardwareMap hwMap) {
        target = 0;
        motorDown = HighMotor.Builder.startBuilding()
                .setMotor(hwMap.get(DcMotorEx.class, shooterMotorDownName))
                .setRunMode(HighMotor.RunMode.Velocity)
                .setReverseMotor(false)
                .setEncoder(true, false)
                .setEncoderResolution(encoderResolutionFly)
                .setWheelDiameter(wheelDiameterFly)
                .useVoltageComensationForVelocity(true)
                .setVelocityPIDCoefficients(kpFly, kiFly, kdFly, kfFly, ksFly, kaFly, 1)
                .setUseZeroPowerBehaviour(false)
                .build();
        motorDown.setTolerance(0.15);

        motorUp = HighMotor.Builder.startBuilding()
                .setMotor(hwMap.get(DcMotorEx.class, shooterMotorUpName))
                .setRunMode(HighMotor.RunMode.Velocity)
                .setReverseMotor(false)
                .setEncoder(true, false)
                .setEncoderResolution(encoderResolutionBack / 2)
                .setWheelDiameter(wheelDiameterBack)
                .useVoltageComensationForVelocity(true)
                .setVelocityPIDCoefficients(kpBack, kiBack, kdBack, kfBack, ksBack, kaBack, 1)
                .setUseZeroPowerBehaviour(false)
                .build();
        motorUp.setTolerance(0.08);
        upTolerance = 0.05;
        downTolerance = 0.1;
    }


    public void setUpTargetVelocity(double velocity) {
        this.targetUp = velocity;
        motorUp.setTarget(velocity);
    }

    public void setDownTargetVelocity(double velocity) {
        this.targetDown = velocity;
        motorDown.setTarget(velocity);
    }

    public void setTargetVelocity(double down, double up) {
        setUpTargetVelocity(up);
        setDownTargetVelocity(down);
    }

    public void setTargetVelocity(double distance) {
        double rawDown = TrajectoryRegression.calculateDown(distance);
        double rawUp = TrajectoryRegression.calculateDown(distance);
        setTargetVelocity(rawDown, rawUp);
    }

    public void setTargetVelocityCompensation(double distance) {
        shouldCompensate = true;
        setTargetVelocity(distance);
    }

    public void setManualVelocity(double velocity) {
        setTargetVelocity(velocity, velocity);
        shouldCompensate = true;
        target = velocity;

    }

    public static double getDownVelocityFromDistance(double x) {
        return TrajectoryRegression.calculateDown(x);
    }

    public static double getUpVelocityFromDistance(double x) {
        return TrajectoryRegression.calculateUp(x);
    }

    public double getVelocityErrorUp() {
        return Math.abs(targetUp - velocityUp);
    }

    public double getVelocityErrorDown() {
        return Math.abs(targetDown - velocityDown);
    }

    public double getVelocityErrorCompensation() {
        return 2 * target - (velocityUp + velocityDown);
    }

    public boolean upAtTarget() {
        return Math.abs(targetUp - velocityUp) <= (upTolerance + upOffset);
    }

    public boolean downAtTarget() {
        return Math.abs(targetDown - velocityDown) <= (downTolerance + downOffset);
    }

    @Override
    public boolean atTarget() {
        return upAtTarget() && downAtTarget();
    }

    public boolean atTargetCompensated() {
        return Math.abs( (targetUp+targetDown) - (velocityUp + velocityDown)) <= (toleranceCompensation+compOffset);
    }

    public void setToleranceCompensationOffset(double offset) {
        this.compOffset = offset;
    }

    public void addToleranceCompensationOffset(double offset) {
        this.compOffset += offset;
    }

    public double getDownTolerance() {
        return downTolerance;
    }

    public double getUpTolerance() {
        return upTolerance;
    }

    public void setUpToleranceOffset(double offset) {
        this.upOffset = offset;
    }

    public void setDownToleranceOffset(double offset) {
        this.downOffset = offset;
    }

    public void addToUpToleranceOffset(double offset) {
        this.upOffset += offset;
    }

    public void addToDownToleranceOffset(double offset) {
        this.downOffset += offset;
    }

    @Override
    public double getTarget() {
        return target;
    }

    public double getTargetDown() {
        return targetDown;
    }

    public double getTargetUp() {
        return targetUp;
    }

    public void setPIDCoefficientsDown(double kp, double kd, double ki, double kf) {
        motorDown.setVelocityPIDCoefficients(kp, ki, kd, kf, 1);
    }

    public void setPIDCoefficientsUp(double kp, double kd, double ki, double kf) {
        motorUp.setVelocityPIDCoefficients(kp, ki, kd, kf, 1);
    }

    public void updateCoefficientsDown() {
        motorDown.setVelocityPIDFSA(kpFly, kiFly, kdFly, kfFly, ksFly, kaFly, 1);
    }

    public void updateCoefficientsUp() {
        motorUp.setVelocityPIDFSA(kpBack, kiBack, kdBack, kfBack, ksBack, kaBack, 1);
    }

    public void setTolerance(double upTolerance, double downTolerance) {
        this.upTolerance = upTolerance;
        this.downTolerance = downTolerance;
    }

    public void updateAllCoefficients() {
        updateCoefficientsDown();
        updateCoefficientsUp();
    }

    public void nanUp() {
        motorUp.setVelocityPIDFSA(0, 0, 0, 0, 0, 0, 1);
    }

    public void nanDown() {
        motorDown.setVelocityPIDFSA(0, 0, 0, 0, 0, 0, 1);
    }

    public void enableCompensation(){
        shouldCompensate = true;
    }

    public void disableCompensation(){
        shouldCompensate = false;
    }

    @Override
    public void update() {
        velocityUp = motorUp.getCurrentVelocity();
        velocityDown = motorDown.getCurrentVelocity();
        if (shouldCompensate) {
            targetUp += getVelocityErrorDown() * kC;
            setUpTargetVelocity(targetUp);
        }
        motorUp.update();
        motorDown.update();
    }
}