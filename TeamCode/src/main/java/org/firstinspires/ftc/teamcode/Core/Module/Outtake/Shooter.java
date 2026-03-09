package org.firstinspires.ftc.teamcode.Core.Module.Outtake;

import static org.firstinspires.ftc.teamcode.Constants.DeviceNames.shooterMotorLeftName;
import static org.firstinspires.ftc.teamcode.Constants.DeviceNames.shooterMotorRightName;
import static org.firstinspires.ftc.teamcode.Constants.OuttakeConstants.ShooterConstants.encoderResolutionFly;
import static org.firstinspires.ftc.teamcode.Constants.OuttakeConstants.ShooterConstants.kaFly;
import static org.firstinspires.ftc.teamcode.Constants.OuttakeConstants.ShooterConstants.kdFly;
import static org.firstinspires.ftc.teamcode.Constants.OuttakeConstants.ShooterConstants.kfFly;
import static org.firstinspires.ftc.teamcode.Constants.OuttakeConstants.ShooterConstants.kiFly;
import static org.firstinspires.ftc.teamcode.Constants.OuttakeConstants.ShooterConstants.kpFly;
import static org.firstinspires.ftc.teamcode.Constants.OuttakeConstants.ShooterConstants.ksFly;
import static org.firstinspires.ftc.teamcode.Constants.OuttakeConstants.ShooterConstants.wheelDiameterFly;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Core.Hardware.HighModule;
import org.firstinspires.ftc.teamcode.Core.Hardware.HighMotor;

@Config
public class Shooter extends HighModule {
    public HighMotor motorRight, motorLeft;

    public double currentVelocity;
    public double tolerance = 0.15, toleranceOffset = 0;
    public double targetVelocity;
    public boolean wasAtTarget = false;
    private double lastVelocity;
    public double jerk = 0;

    public static double maxVelocity = 7.2;

    public Shooter(HardwareMap hwMap) {
        targetVelocity = 0;
        motorLeft = HighMotor.Builder.startBuilding()
                .setMotor(hwMap.get(DcMotorEx.class, shooterMotorLeftName))
                .setRunMode(HighMotor.RunMode.Velocity)
                .setReverseMotor(true)
                .setEncoder(true, false)
                .setEncoderResolution(encoderResolutionFly)
                .setWheelDiameter(wheelDiameterFly)
                .useVoltageComensationForVelocity(true)
                .setVelocityPIDCoefficients(kpFly, kiFly, kdFly, kfFly, ksFly, kaFly, 1)
                .setUseZeroPowerBehaviour(false)
                .build();

        motorLeft.pidfVelocity.setFilterGain(0.8);
        motorLeft.setTolerance(tolerance);
        motorRight = HighMotor.Builder.startBuilding()
                .setMotor(hwMap.get(DcMotorEx.class, shooterMotorRightName))
                .setRunMode(HighMotor.RunMode.Standard)
                .setReverseMotor(false)
                .setUseZeroPowerBehaviour(false)
                .build();
    }

    public void setTargetVelocity(double velocity) {
        velocity = getFlyWheelVelocity(velocity);
        this.targetVelocity = Range.clip(velocity, 0, maxVelocity);
        motorLeft.setTarget(this.targetVelocity);
    }
    public double getFlyWheelVelocity(double v0){
        return v0*2*1.25;
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
        motorLeft.setVelocityPIDCoefficients(kp, ki, kd, kf, 1);
    }

    public void updateCoefficients() {
        motorLeft.setVelocityPIDFSA(kpFly, kiFly, kdFly, kfFly, ksFly, kaFly, 1);
    }

    public void setTolerance(double newTolerance) {
        this.tolerance = newTolerance;
        motorLeft.setTolerance(newTolerance);
    }

    public void nanMotors() {
        motorLeft.setVelocityPIDFSA(0, 0, 0, 0, 0, 0, 1);
    }

    @Override
    public void update() {
        currentVelocity = motorLeft.getCurrentVelocity();
        jerk = Math.abs(lastVelocity - currentVelocity);
        lastVelocity = currentVelocity;
        motorLeft.update();
        double appliedPower = motorLeft.motor.getPower();
        motorRight.setPower(appliedPower);
        motorRight.update();
    }
}