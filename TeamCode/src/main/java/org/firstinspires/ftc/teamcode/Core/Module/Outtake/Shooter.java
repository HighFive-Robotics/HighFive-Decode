package org.firstinspires.ftc.teamcode.Core.Module.Outtake;

import static org.firstinspires.ftc.teamcode.Constants.DeviceNames.shooterMotorDownName;
import static org.firstinspires.ftc.teamcode.Constants.DeviceNames.shooterMotorUpName;
import static org.firstinspires.ftc.teamcode.Constants.OuttakeConstants.ShooterBackWheelParams.encoderResolutionBack;
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
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Core.Hardware.HighModule;
import org.firstinspires.ftc.teamcode.Core.Hardware.HighMotor;

@Config
public class Shooter extends HighModule {
    public HighMotor motorUp, motorDown;
    public double velocityUp, velocityDown, upTolerance, downTolerance, upOffset = 0.05, downOffset = 0.1;
    public double targetUp, targetDown;
    public static double massBall = 0.050;
    public static double massTopWheel = 0.030;
    public static double couplingEfficiency = 0.8;
    public static double alpha = 0.5;
    public static boolean shouldUsePhysics = false;

    public Shooter(HardwareMap hwMap) {
        motorDown = HighMotor.Builder.startBuilding()
                .setMotor(hwMap.get(DcMotorEx.class, shooterMotorDownName))
                .setRunMode(HighMotor.RunMode.Velocity)
                .setReverseMotor(false)
                .setEncoder(true, false)
                .setEncoderResolution(encoderResolutionFly)
                .setMotorRPM(HighMotor.MotorRPM.RPM6000)
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
                .setEncoderResolution(encoderResolutionBack)
                .setMotorRPM(HighMotor.MotorRPM.RPM6000)
                .setWheelDiameter(wheelDiameterBack)
                .useVoltageComensationForVelocity(true)
                .setVelocityPIDCoefficients(kpBack, kiBack, kdBack, kfBack, ksBack, kaBack, 1)
                .setUseZeroPowerBehaviour(false)
                .build();
        motorUp.setTolerance(0.08);
        upTolerance = 0.05;
        downTolerance = 0.1;
    }

    public void setAntiBackSpinVelocity(double velocity) {
        double velocityUp;
        if (velocity >= 3.2) {
            velocityUp = -scaleWithDecay(velocity);
        } else {
            velocityUp = scaleWithDecay(velocity);
        }
        this.targetDown = velocity;
        this.targetUp = velocityUp;
        motorUp.setTarget(velocityUp);
        motorDown.setTarget(velocity);
    }

    public void setVelocityPhysics(double targetLinearVelocity, double compensation) {
        double legacyUpVelocity = scaleWithDecayRate(targetLinearVelocity);
        double legacyExitEnergy = (targetLinearVelocity + Math.abs(legacyUpVelocity)) / 2.0;
        double legacySpin = targetLinearVelocity - Math.abs(legacyUpVelocity);
        double targetSpin = legacySpin * (1.0 - alpha);
        double theoreticalDown = legacyExitEnergy + (targetSpin / 2.0);
        double theoreticalUp = legacyExitEnergy - (targetSpin / 2.0);
        double massRatio = massBall / massTopWheel;
        double momentumFactor = 1.0 + (massRatio * couplingEfficiency);
        double commandUp = theoreticalUp * momentumFactor;
        double extraEnergyCost = (commandUp - theoreticalUp) / compensation;
        double commandDown = theoreticalDown;
        if (legacyUpVelocity < 0) {
            commandDown += revertScale(Math.abs(legacyUpVelocity)) / compensation;
        } else {
            commandDown += decayedToExtension(legacyUpVelocity) / compensation;
        }
        commandDown -= extraEnergyCost;
        this.targetUp = commandUp;
        this.targetDown = commandDown;
        motorUp.setTarget(this.targetUp);
        motorDown.setTarget(this.targetDown);
    }

    public void setVelocity(double velocity, double compensation) {
        if (shouldUsePhysics) {
            double velocityUp = scaleWithDecayRate(velocity);
            double velocityDown = velocity;
            if (velocityUp < 0) {
                velocityDown += revertScale(Math.abs(velocityUp)) / compensation;
            } else {
                velocityDown += decayedToExtension(velocityUp) / compensation;
            }
            this.targetUp = velocityUp;
            this.targetDown = velocityDown;
            motorUp.setTarget(velocityUp);
            motorDown.setTarget(velocityDown);
        } else setVelocityPhysics(velocity, compensation);
    }

    public void setVelocity(double velocity) {
        setVelocity(velocity, 1);
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
        setUpTargetVelocity(getUpVelocityFromDistance(distance));
        setDownTargetVelocity(getDownVelocityFromDistance(distance));
    }

    public static double scaleWithDecay(double x) {
        double slope = 0.20 / 1.23;
        double hingeValue = 2.0 * slope;
        double target = -0.1;
        double exponent = 4 + x;

        if (x <= 2.0) {
            return x * slope;
        }
        double progress = (x - 2.0) / (7.2 - 2.0);
        return hingeValue + (target - hingeValue) * Math.pow(progress, exponent);
    }

    public static double scaleWithDecayRate(double x) {
        if (x <= 3) {
            return 0.066 * x;
        } else {
            return (((((((0.0005849323 * x - 0.01907369) * x + 0.2549253) * x - 1.800606) * x + 7.238965) * x - 16.59063) * x + 19.87657) * x - 9.144974);
        }
    }

    public static double revertScale(double x) {
        return x / 0.17988;
    }

    public double decayedToExtension(double x) {
        return x * x + (x / (0.066 * Math.pow(2, x))) - 7.6 * x;
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

    @Override
    public void update() {
        velocityUp = motorUp.getCurrentVelocity();
        velocityDown = motorDown.getCurrentVelocity();
        motorUp.update();
        motorDown.update();
    }
}