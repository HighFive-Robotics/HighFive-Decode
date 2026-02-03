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
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Core.Hardware.HighModule;
import org.firstinspires.ftc.teamcode.Core.Hardware.HighMotor;
import org.opencv.core.Mat;

@Config
public class Shooter extends HighModule {
    public HighMotor motorUp, motorDown;
    public Blocker blocker;
    public double velocityUp, velocityDown, upTolerance , downTolerance;
    public double targetUp, targetDown;
    public static double massBall = 0.050;
    public static double massTopWheel = 0.030;
    public static double couplingEfficenty = 0.8;
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
        blocker = new Blocker(hwMap, Blocker.OpenPosition, true);
        upTolerance = 0.05;
        downTolerance = 0.1;
    }

    public void setAntiBackSpinVelocity(double velocity) {
        if (velocity >= 3.2) {
            double veloUp = -scaleWithDecay(velocity);
            double veloDown = velocity + Math.abs(veloUp) * 1.5;
            this.targetDown = velocity;
            this.targetUp = veloUp;
            motorUp.setTarget(veloUp);
            motorDown.setTarget(velocity);
        } else {
            double veloUp = scaleWithDecay(velocity);
            this.targetDown = velocity;
            this.targetUp = veloUp;
            motorUp.setTarget(veloUp);
            motorDown.setTarget(velocity);
        }
    }
    public void setVelocityPhysics(double targetLinearVelocity, double compensation) {
        double legacyUpVelo = scaleWithDecayRate(targetLinearVelocity);
        double legacyDownVelo = targetLinearVelocity;
        double legacyExitEnergy = (legacyDownVelo + Math.abs(legacyUpVelo)) / 2.0;
        double legacySpin = legacyDownVelo - Math.abs(legacyUpVelo);
        double targetSpin = legacySpin * (1.0 - alpha);
        double theoreticalDown = legacyExitEnergy + (targetSpin / 2.0);
        double theoreticalUp = legacyExitEnergy - (targetSpin / 2.0);
        double massRatio = massBall / massTopWheel;
        double momentumFactor = 1.0 + (massRatio * couplingEfficenty);
        double commandUp = theoreticalUp * momentumFactor;
        double extraEnergyCost = (commandUp - theoreticalUp) / compensation;
        double commandDown = theoreticalDown;
        if (legacyUpVelo < 0) {
            commandDown += revertScale(Math.abs(legacyUpVelo)) / compensation;
        } else {
            commandDown += decayedToExtension(legacyUpVelo) / compensation;
        }
        commandDown -= extraEnergyCost;
        this.targetUp = commandUp;
        this.targetDown = commandDown;
        motorUp.setTarget(this.targetUp);
        motorDown.setTarget(this.targetDown);
    }
    public void setVelocity(double velocity, double compensation) {
        if(shouldUsePhysics){
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
        }else setVelocityPhysics(velocity,compensation);
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
    public void setTargetVelocity(double distance){
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

    public static double decayedToExtension(double x) {
        return x * x + (x / (0.066 * Math.pow(2, x))) - 7.6 * x;
    }
    public static double getDownVelocityFromDistance(double x) {
        double q = -1.2214742e-21;
        double w =  2.2340637e-18;
        double r = -1.8076462e-15;
        double t =  8.5336681e-13;
        double o = -2.6006502e-10;
        double h =  5.3215045e-8;
        double a = -0.0000073453731;
        double b =  0.00066909674;
        double c = -0.038158381;
        double d =  1.2235733;
        double f = -14.12222;
        double out = (((((((((q * x + w) * x + r) * x + t) * x + o) * x + h) * x + a) * x + b) * x + c) * x + d) * x + f;
        out = Range.clip(out, -7.2, 7.2);
        return out;
    }
    public static double getUpVelocityFromDistance(double x) {
        double q = -3.45907498e-23;
        double w =  1.12139456e-19;
        double r = -1.24832445e-16;
        double t =  7.05167915e-14;
        double o = -2.32198243e-11;
        double h =  4.72528627e-9;
        double a = -6.05676656e-7;
        double b =  0.0000483669971;
        double c = -0.00229899239;
        double d =  0.058735351;
        double f = -0.365053577;
        double out = (((((((((q * x + w) * x + r) * x + t) * x + o) * x + h) * x + a) * x + b) * x + c) * x + d) * x + f;
        out = Range.clip(out , -1 , 1);
        return out;
    }
    public double getVelocityErrorUp() {
        return Math.abs(targetUp - velocityUp);
    }

    public double getVelocityErrorDown() {
        return Math.abs(targetDown - velocityDown);
    }

    public boolean upAtTarget() {
        return Math.abs(targetUp - velocityUp) <= upTolerance;
    }

    public boolean downAtTarget() {
        return Math.abs(targetDown - velocityDown) <= downTolerance;
    }

    public boolean upAtTarget(double tolerance) {
        return Math.abs(targetUp - velocityUp) <= tolerance;
    }

    public boolean downAtTarget(double tolerance) {
        return Math.abs(targetDown - velocityDown) <= tolerance;
    }

    @Override
    public boolean atTarget() {
        return upAtTarget() && downAtTarget();
    }

    public boolean atTarget(double tolerance) {
        return upAtTarget(tolerance) && downAtTarget(tolerance);
    }

    @Override
    public double getTarget() {
        return target;
    }

    public void setPIDCoefDown(double kp, double kd, double ki, double kf) {
        motorDown.setVelocityPIDCoefficients(kp, ki, kd, kf, 1);
    }

    public void setPIDCoefUp(double kp, double kd, double ki, double kf) {
        motorUp.setVelocityPIDCoefficients(kp, ki, kd, kf, 1);
    }

    public void updateCoefDown() {
        motorDown.setVelocityPIDFSA(kpFly, kiFly, kdFly, kfFly, ksFly, kaFly, 1);
    }

    public void updateCoefUp() {
        motorUp.setVelocityPIDFSA(kpBack, kiBack, kdBack, kfBack, ksBack, kaBack, 1);
    }
    public void setTolerance(double upTolerance , double downTolerance){
        this.upTolerance = upTolerance;
        this.downTolerance = downTolerance;
    }
    public void updateAllCoef() {
        updateCoefDown();
        updateCoefUp();
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
        blocker.update();
    }
}