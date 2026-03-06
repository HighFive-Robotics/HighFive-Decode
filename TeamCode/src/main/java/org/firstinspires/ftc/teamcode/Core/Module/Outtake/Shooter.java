package org.firstinspires.ftc.teamcode.Core.Module.Outtake;

import static org.firstinspires.ftc.teamcode.Constants.DeviceNames.shooterMotorDownName;
import static org.firstinspires.ftc.teamcode.Constants.DeviceNames.shooterMotorUpName;
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
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Core.Hardware.HighModule;
import org.firstinspires.ftc.teamcode.Core.Hardware.HighMotor;

@Config
public class Shooter extends HighModule {
    public HighMotor motorUp, motorDown;

    public double currentVelocity;
    public double tolerance = 0.15, toleranceOffset = 0;
    public double targetVelocity;
    public boolean wasAtTarget = false;
    private double lastVelocity;
    public double jerk = 0;

    public static double maxVelocity = 7.2;

    public Shooter(HardwareMap hwMap) {
        targetVelocity = 0;
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

        motorDown.pidfVelocity.setFilterGain(0.8);
        motorDown.setTolerance(tolerance);
        motorUp = HighMotor.Builder.startBuilding()
                .setMotor(hwMap.get(DcMotorEx.class, shooterMotorUpName))
                .setRunMode(HighMotor.RunMode.Standard)
                .setReverseMotor(false)
                .setUseZeroPowerBehaviour(false)
                .build();
    }

    public void setTargetVelocity(double velocity) {
        this.targetVelocity = Range.clip(velocity, 0, maxVelocity);
        motorDown.setTarget(this.targetVelocity);
    }

    public double getVelocityError() {
        return Math.abs(targetVelocity - currentVelocity);
    }

    @Override
    public boolean atTarget() {
        return getVelocityError() <= (tolerance + toleranceOffset);
    }

    public void setToleranceOffset(double offset) {
        this.toleranceOffset = offset;
    }

    public void addToleranceOffset(double offset) {
        this.toleranceOffset += offset;
    }

    @Override
    public double getTarget() {
        return targetVelocity;
    }

    public void setPIDCoefficients(double kp, double kd, double ki, double kf) {
        motorDown.setVelocityPIDCoefficients(kp, ki, kd, kf, 1);
    }

    public void updateCoefficients() {
        motorDown.setVelocityPIDFSA(kpFly, kiFly, kdFly, kfFly, ksFly, kaFly, 1);
    }

    public void setTolerance(double newTolerance) {
        this.tolerance = newTolerance;
        motorDown.setTolerance(newTolerance);
    }

    public void nanMotors() {
        motorDown.setVelocityPIDFSA(0, 0, 0, 0, 0, 0, 1);
    }

    @Override
    public void update() {
        currentVelocity = motorDown.getCurrentVelocity();
        jerk = Math.abs(lastVelocity - currentVelocity);
        lastVelocity = currentVelocity;
        motorDown.update();
        double appliedPower = motorDown.motor.getPower();
        motorUp.setPower(appliedPower);
        motorUp.update();
    }
}