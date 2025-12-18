/*
 * Copyright (c) 2020-2025 High Five (http://www.highfive.ro)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software,
 * and to permit others to do the same.
 *
 * This permission is granted under the condition that the above copyright notice
 * and this permission notice are included in all copies or significant portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED AS IS, WITHOUT ANY WARRANTIES OF ANY KIND, EITHER EXPRESS OR IMPLIED.
 * THIS INCLUDES, BUT IS NOT LIMITED TO, WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE,
 * OR NON-INFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR THE COPYRIGHT HOLDERS BE HELD LIABLE FOR ANY CLAIMS, DAMAGES,
 * OR OTHER LIABILITIES THAT MAY ARISE FROM THE USE OF THE SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.Core.Hardware;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Core.Algorithms.AsymmetricMotionProfiler;
import org.firstinspires.ftc.teamcode.Core.Algorithms.SQUIDAngle;


public class HighServo {

    public enum RunMode {
        MotionProfiler,
        Standard,
        ContinuousRotation,
        PIDAngle
    }

    public enum FeedForwardType{
        Arm, /// if we want to use feedforward for an arm
        Lift  ///if we want to use feedforward for a lift
    }

    public Servo servo;
    public CRServo CRServo;
    public AnalogInput analogInput;
    private final AsymmetricMotionProfiler motionProfiler = new AsymmetricMotionProfiler(1, 1, 1);
    public final SQUIDAngle pidfController = new SQUIDAngle(0,0,0,0);
    public FeedForwardType feedForwardType;
    public HighEncoder encoder;
    private double targetPosition, targetPositionMotionProfile, currentPosition, lastPosition = -1;
    private final double epsilon = 1e-5;
    private boolean useAnalogInput = false, atTarget = false;
    private double error = epsilon, voltage, minPosition = 0, maxPosition = 1, minVoltage = 0, maxVoltage = 3.3;
    private double power, lastPower = -2;
    public boolean inInit = true;
    private double target = 0, tolerance = epsilon;
    private double currentPositionPID = 0, maxPIDPower = 1, kF = 0, initialAngle = 0, ticksPerDegree = 0;
    private double encoderResolution = 1;
    public ElapsedTime timer = new ElapsedTime();
    public double time = -1;

    public RunMode runMode;

    /**
     * This constructor is used just for the ContinuousRotation runMode,
     * where you don't need an initial position.
     *
     * @param CRServo gives the ContinuousRotation servo we will work with
     * @param runMode gives the runMode ( ContinuousRotation )
     */
    public HighServo(CRServo CRServo, RunMode runMode) {
        this.CRServo = CRServo;
        this.runMode = runMode;
    }

    public HighServo(CRServo CRServo, HighEncoder encoder, RunMode runMode) {
        this.encoder = encoder;
        this.CRServo = CRServo;
        this.runMode = runMode;
    }

    public HighServo(){

    }

    /**
     * This constructor is used for the MotionProfiler or Standard runModes,
     * where you need an initial position.
     *
     * @param servo           gives the servo we will work with
     * @param runMode         gives the runMode ( MotionProfiler or Standard )
     * @param initialPosition gives the init position of the servo
     * @param isAuto    knows if the servo should init or not
     */
    public HighServo(Servo servo, RunMode runMode, double initialPosition, boolean isAuto) {
        this.servo = servo;
        this.runMode = runMode;
        setInitialPosition(initialPosition, isAuto);
        if (runMode == RunMode.MotionProfiler) {
            motionProfiler.setMotion(initialPosition, initialPosition);
        }
    }

    /**
     * This constructor is for when we want to use the analog input
     * of the servo.
     *
     * @param servo           gives the servo we will work with
     * @param analogInput     this is the analog input of the axon servo,
     *                        using this you must use this method
     *                        setAnalogInputCoefficients();
     * @param runMode         gives the runMode
     * @param initialPosition gives the init position of the servo
     * @param isAuto    knows if the servo should init or not
     */
    public HighServo(Servo servo, AnalogInput analogInput, RunMode runMode, double initialPosition, boolean isAuto) {
        this.servo = servo;
        this.analogInput = analogInput;
        useAnalogInput = true;
        this.runMode = runMode;
        setInitialPosition(initialPosition, isAuto);
        if (runMode == RunMode.MotionProfiler) {
            motionProfiler.setMotion(initialPosition, initialPosition);
        }
    }

    /**
     * This method is called to set the initial position on the servo.
     * For MotionProfiler and Standard runModes we set the init position.
     * For the ContinuousRotation runMode we don't set any power for init.
     *
     * @param position     gives the init position of the servo
     * @param isAutonomous knows if the servo should init or not
     */
    private void setInitialPosition(double position, boolean isAutonomous) {
        inInit = false;
        targetPosition = position;
        if (isAutonomous) {
            this.currentPosition = position;
            servo.setPosition(targetPosition);
        }
    }


    /**
     * This method is called just for the MotionProfiler runMode
     * so we can set the coefficients needed for the motion profiler
     *
     * @param maxVelocity  gives the max velocity that the motion
     *                     profiler is allowed to reach
     * @param acceleration gives the acceleration which defines how
     *                     quickly the system speeds up from rest to
     *                     max velocity
     * @param deceleration gives the deceleration which determines how
     *                     quickly the system slows down before reaching
     *                     the target position
     */
    public void setMotionProfilerCoefficients(double maxVelocity, double acceleration, double deceleration) {
        motionProfiler.setCoefficients(maxVelocity, acceleration, deceleration);
    }

    /**
     * This method is called to set the position on the servo, only for Motion Profiler and Standard runModes.
     * For MotionProfiler runMode we set the target position using motionProfiler.setMotion().
     * For Standard runMode we simply set the target position.
     *
     * @param position gives the wanted position of the servo
     */
    public void setPosition(double position) {
        inInit = false;
        switch (runMode) {
            case MotionProfiler:
                lastPosition = targetPosition;
                targetPosition = position;
                targetPositionMotionProfile = position;
                motionProfiler.setMotion(lastPosition, targetPositionMotionProfile);
                break;
            case Standard:
                targetPosition = position;
                setInitialPosition(position,true);
                break;
        }
        atTarget = false;
    }

    /**
     * This method is called only for Motion Profiler and Standard runModes,to set the position on the
     * servo, while also using a time to determine if the servo reached the wanted position.
     * For MotionProfiler runMode we set the target position using motionProfiler.setMotion();
     * For Standard runMode we simply set the target position.
     *
     * @param position gives the wanted position of the servo
     * @param time     the time after the servo thinks that it reached the target position
     */
    public void setPosition(double position, double time) {
        switch (runMode) {
            case MotionProfiler:
                targetPosition = position;
                targetPositionMotionProfile = position;
                motionProfiler.setMotion(lastPosition, targetPositionMotionProfile);
                break;
            case Standard:
                targetPosition = Range.clip(position, 0, 1);
                break;
        }
        atTarget = false;
        this.time = time;
        timer.reset();
    }

    /**
     * This method is called when we use the analog input so we can set the coefficients needed
     * for the calculatePose() method.
     *
     * @param error       gives the error that we accept
     * @param minVoltage  gives the minimum voltage that can be read from the analog input
     * @param maxVoltage  gives the maximum voltage that can be read from the analog input
     * @param minPosition gives the minimum position the servo can take
     * @param maxPosition gives the maximum position the servo can take
     */
    public void setAnalogInputCoefficients(double error, double minVoltage, double maxVoltage, double minPosition, double maxPosition) {
        this.error = error;
        this.minVoltage = minVoltage;
        this.maxVoltage = maxVoltage;
        this.minPosition = minPosition;
        this.maxPosition = maxPosition;
    }

    /**
     * This method is used to calculate the position of the axon servo,
     * reading the voltage and calculating the position.
     *
     * @param voltage this is the voltage read from the analog input.
     * @return this method returns the position of the servo.
     */
    private double calculatePose(double voltage) {
        return Range.clip(Range.scale(voltage, minVoltage, maxVoltage, minPosition, maxPosition), minPosition, maxPosition);
    }


    /**
     * @return this method returns the wanted target position
     */
    public double getTarget() {
        return targetPosition;
    }

    /**
     * @return this method returns the current position
     */
    public double getCurrentPositionPID() {
        return currentPositionPID;
    }

    /**
     * @return this method returns the voltage read from the analog input
     */
    public double getVoltage() {
        return voltage;
    }

    /**
     * @return this method returns true if the servo is at the desired position or false if not
     */
    public boolean atTarget() {
        return atTarget;
    }

    /**
     * This method is used to set the power given to the ContinuousRotation Servo
     *
     * @param power gives the power the servo should have
     */
    public void setPower(double power) {
        this.power = Range.clip(power, -1, 1);
    }

    /**
     * @return this method returns the power that the ContinuousRotation Servo currently has
     */
    public double getPower() {
        return power;
    }

    /**
     *
     * @return this method returns the target
     */
    public double getTargetPID() {
        return target;
    }

    /**
     *
     * This method sets the target by setting the SetPoint for the PIDFController.
     *
     * @param target gives the value of the wanted target
     */
    public void setTarget(double target) {
        this.target = target;
        pidfController.setSetPoint(target);
    }

    /**
     *
     * @return this method returns true if the motor is at the desired position or false if not*
     * We return true if the absolute value of the difference between the wanted target and the current
     * position is smaller then the accepted error (tolerance) or false if not.
     */
    public boolean atTargetPID() {
        return Math.abs(target - currentPositionPID) <= tolerance;
    }

    public double getTolerance() {
        return tolerance;
    }

    /**
     * This method sets the tolerance (the accepted error).
     *
     * @param tolerance gives the value of the tolerance
     */
    public void setTolerance(double tolerance) {
        this.tolerance = tolerance;
        pidfController.setTolerance(tolerance);
    }

    /**
     *
     * @return this method returns the initial angle of the arm used in feedforward calculations
     */
    public double getInitialAngle(){
        return initialAngle;
    }

    /**
     *
     * @return this method returns how many encoder ticks correspond to one degree of movement.
     */
    public double getTicksPerDegree(){
        return ticksPerDegree;
    }

    /**
     * This method sets how many encoder ticks correspond to one degree of movement.
     */
    public void setTicksPerDegree(double ticksPerDegree){
        this.ticksPerDegree = ticksPerDegree;
    }

    /**
     * This method sets the maximum power our PID controller is allowed to apply.
     *
     * @param maxPIDPower the value of the maximum power our PID controller is allowed to apply
     */
    public void setMaxPIDPower(double maxPIDPower){
        this.maxPIDPower = maxPIDPower;
    }

    /**
     *
     * @return this method returns the feedforward type (Arm or Lift)
     */
    public FeedForwardType getFeedForwardType(){
        return feedForwardType;
    }

    /**
     * This method sets the PID coefficients (kP, kI, kD) for the PID controller.
     * This method is used when we only want to configure the basic proportional,
     * integral, and derivative gains, and no feedforward (kF = 0).
     *
     * @param kP the proportional gain
     * @param kI the integral gain
     * @param kD the derivative gain
     */
    public void setPIDCoefficients(double kP, double kI, double kD) {
        pidfController.setPIDF(kP, kI, kD, 0);
    }

    /**
     * This method sets the PID coefficients (kP, kI, kD) for the PID controller.
     * This method is used when we only want to configure the basic proportional,
     * integral, and derivative gains, and no feedforward (kF = 0).
     * Also it sets the maximum power that the PID controller is allowed to apply.
     *
     * @param kP the proportional gain
     * @param kI the integral gain
     * @param kD the derivative gain
     * @param maxPIDPower the value of the maximum power our PID controller is allowed to apply
     */
    public void setPIDCoefficients(double kP, double kI, double kD, double maxPIDPower) {
        this.maxPIDPower = Math.abs(maxPIDPower);
        pidfController.setPIDF(kP, kI, kD, 0);
    }

    /**
     * Sets the PID coefficients (kP, kI, kD) along with feedforward gain (kF), the feed forward type(Arm or Lift),
     * initial angle and ticks per degree.
     *
     * @param kP the proportional gain
     * @param kI the integral gain
     * @param kD the derivative gain
     * @param kF the feedforward gain
     * @param feedForwardType the feedforward type(Arm or Lift)
     * @param initialAngle the initial angle
     * @param ticksPerDegree gives how many encoder ticks correspond to one degree of movement
     */
    public void setPIDCoefficients(double kP, double kI, double kD, double kF, FeedForwardType feedForwardType, double initialAngle, double ticksPerDegree) {
        this.kF = kF;
        this.feedForwardType = feedForwardType;
        this.initialAngle = initialAngle;
        this.ticksPerDegree = ticksPerDegree;
        pidfController.setPIDF(kP, kI, kD, 0);
    }

    /**
     * Sets the PID coefficients (kP, kI, kD) along with feedforward gain (kF), the feed forward type(Arm or Lift),
     * initial angle, ticks per degree and the maximum power our PID controller is allowed to apply
     *
     *
     * @param kP the proportional gain
     * @param kI the integral gain
     * @param kD the derivative gain
     * @param kF the feedforward gain
     * @param feedForwardType the feedforward type(Arm or Lift)
     */
    public void setPIDCoefficients(double kP, double kI, double kD, double kF, FeedForwardType feedForwardType) {
        this.kF = kF;
        this.feedForwardType = feedForwardType;
        pidfController.setPIDF(kP, kI, kD, 0);
    }

    /**
     * Sets the PID coefficients (kP, kI, kD) along with feedforward gain (kF), the feed forward type(Arm or Lift),
     * initial angle, ticks per degree and the maximum power our PID controller is allowed to apply
     *
     *
     * @param kP the proportional gain
     * @param kI the integral gain
     * @param kD the derivative gain
     * @param kF the feedforward gain
     * @param feedForwardType the feedforward type(Arm or Lift)
     * @param maxPIDPower the value of the maximum power our PID controller is allowed to apply
     */
    public void setPIDCoefficients(double kP, double kI, double kD, double kF, FeedForwardType feedForwardType, double maxPIDPower) {
        this.kF = kF;
        this.feedForwardType = feedForwardType;
        this.maxPIDPower = Math.abs(maxPIDPower);
        pidfController.setPIDF(kP, kI, kD, 0);
    }

    /**
     * Sets the PID coefficients (kP, kI, kD) along with feedforward gain (kF), the feed forward type(Arm or Lift),
     * initial angle, ticks per degree and the maximum power our PID controller is allowed to apply
     *
     *
     * @param kP the proportional gain
     * @param kI the integral gain
     * @param kD the derivative gain
     * @param kF the feedforward gain
     * @param feedForwardType the feedforward type(Arm or Lift)
     * @param initialAngle the initial angle
     * @param ticksPerDegree gives how many encoder ticks correspond to one degree of movement
     * @param maxPIDPower the value of the maximum power our PID controller is allowed to apply
     */
    public void setPIDCoefficients(double kP, double kI, double kD, double kF, FeedForwardType feedForwardType, double initialAngle, double ticksPerDegree , double maxPIDPower) {
        this.kF = kF;
        this.feedForwardType = feedForwardType;
        this.initialAngle = initialAngle;
        this.ticksPerDegree = ticksPerDegree;
        this.maxPIDPower = Math.abs(maxPIDPower);
        pidfController.setPIDF(kP, kI, kD, 0);

    }

    /**
     * This method resets the PID. This clears any accumulated error.
     */
    public void resetPID() {
        pidfController.reset();
    }

    /**
     *
     * This method calculates the total motor power using a combination of PID control and feedforward compensation.*
     * First of all we calculate the PID power based on the current position and target. Then we calculate the
     * feedforward depending on the feedforward type (Arm or Lift).
     * We calculate the power needed to hold or move the arm/lift based on its angular position.
     * In the end, the total output( PID power + FeedForward ) is clipped to the maximum allowed PID power.
     *
     * @param currentPosition gives the current position
     * @return this method returns the final power to be applied to the motor, in the range [-maxPIDPower, maxPIDPower]
     */
    public double getPowerPID(double currentPosition) {
        this.currentPosition = currentPosition;
        double PidPower = pidfController.calculate(currentPosition);
        return Range.clip(PidPower, -maxPIDPower, maxPIDPower);
    }

    public void setEncoderResolution(double encoderResolution) {
        this.encoderResolution = encoderResolution;
    }

    /**
     * This method updates the servo status based on the current runMode:
     * For MotionProfiler runMode, it gets the target position from the MotionProfiler and sets the
     * servo position accordingly.
     * For Standard runMode, it sets the servo position only if it's not already on that position (the
     * target position differs from the last position by more than epsilon).
     * For ContinuousRotation, it updates the servo power only if it has changed since the last update.(the
     * target position differs from the last position by more than epsilon).
     * If analog input is true, it reads the voltage, calculates the current position, and decides if
     * the servo reached the target position within an error margin.
     * It also marks if the servo reached the target position if the timer exceeds the allowed time.
     */
    public void update() {
        if (timer.milliseconds() >= time && time != -1) {
            atTarget = true;
            time = -1;
        } else if (useAnalogInput) {
            voltage = analogInput.getVoltage();
            currentPosition = calculatePose(voltage);
            if (Math.abs(targetPosition - currentPosition) <= error) {
                atTarget = true;
            }
        }
        switch (runMode) {
            case MotionProfiler:
                targetPositionMotionProfile = motionProfiler.getPosition();
                servo.setPosition(targetPositionMotionProfile);
                lastPosition = targetPositionMotionProfile;
                break;
            case Standard:
                if(!inInit) {
                    if (Math.abs(targetPosition - lastPosition) >= epsilon) {
                        servo.setPosition(targetPosition);
                        lastPosition = targetPosition;
                    }
                }
                break;
            case ContinuousRotation:
                if (Math.abs(power - lastPower) >= epsilon) {
                    CRServo.setPower(power);
                    lastPower = power;
                }
                break;
            case PIDAngle:
                currentPositionPID = encoder.getPosition();
                currentPositionPID = currentPositionPID % encoderResolution;
                if (currentPositionPID < 0) currentPositionPID += encoderResolution;
                currentPositionPID = currentPositionPID / encoderResolution * 360;
                power = pidfController.calculate(currentPositionPID);
                if (Math.abs(power - lastPower) >= epsilon) {
                    CRServo.setPower(power);
                    lastPower = power;
                }
                atTarget = pidfController.atSetPoint();
                break;
        }
    }

    /**
     * This method sends various variables to the Driver Hub to facilitate easier debugging.
     *
     * @param telemetry this is the telemetry that is going to be used
     */
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Run mode: ", runMode);
        switch (runMode) {
            case MotionProfiler:
                telemetry.addData("Target position: ", targetPosition);
                telemetry.addData("Current position: ", currentPosition);
                telemetry.addData("Last position: ", lastPosition);
                telemetry.addData("Timer time : ", timer.milliseconds());
                telemetry.addData("Target time: ", time);
                telemetry.addData("At target: ", atTarget);
                motionProfiler.telemetry(telemetry);
                if (useAnalogInput) {
                    telemetry.addData("Error: ", error);
                    telemetry.addData("Voltage: ", voltage);
                    telemetry.addData("Minimum position", minPosition);
                    telemetry.addData("Maximum position", maxPosition);
                    telemetry.addData("Minimum voltage", minVoltage);
                    telemetry.addData("Maximum voltage", maxVoltage);
                }
                break;
            case Standard:
                telemetry.addData("Target position: ", targetPosition);
                telemetry.addData("Current position: ", currentPosition);
                telemetry.addData("Last position: ", lastPosition);
                telemetry.addData("Timer time : ", timer.milliseconds());
                telemetry.addData("Target time: ", time);
                telemetry.addData("At target: ", atTarget);
                if (useAnalogInput) {
                    telemetry.addData("At target: ", atTarget);
                    telemetry.addData("Error: ", error);
                    telemetry.addData("Voltage: ", voltage);
                    telemetry.addData("Minimum position", minPosition);
                    telemetry.addData("Maximum position", maxPosition);
                    telemetry.addData("Minimum voltage", minVoltage);
                    telemetry.addData("Maximum voltage", maxVoltage);
                }
                break;
            case ContinuousRotation:
                telemetry.addData("Current power: ", power);
                telemetry.addData("Last power: ", lastPower);
                break;
        }
    }
    /*
       |
       | BUILDER CLASS
       |
     */

    public interface ServoSetStep{
        public RunModeStep setServo(Servo servo);
        public RunModeStep setServo(CRServo servo);
    }
    public interface RunModeStep{
        public Builder setContinousRotationRunMode();
        public MotionProfilerRunMode setMotionProfilerRunMode();
        public Builder setStandardRunMode();
        public Builder setPIDRunMode();
    }
    public interface StandardRunMode{
        public Builder setAnalogInput(AnalogInput analogInput);
        public Builder setAnalogInputCoefficients(double error, double minVoltage, double maxVoltage, double minPosition, double maxPosition);;
    }
    public interface MotionProfilerRunMode{
        public Builder setMotionProfilerCoefficients(double maxVelocity, double acceleration, double deceleration);
    }


    public static class Builder implements
            ServoSetStep,
            RunModeStep,
            StandardRunMode,
            MotionProfilerRunMode{
        public HighServo servo = new HighServo();
        private Builder(){
        }
        public static ServoSetStep startBuilding(){
            return new Builder();
        }
        @Override
        public RunModeStep setServo(Servo servo) {
            this.servo.servo = servo;
            return this;
        }
        @Override
        public RunModeStep setServo(CRServo servo) {
            this.servo.CRServo = servo;
            return this;
        }

        @Override
        public Builder setContinousRotationRunMode() {
            this.servo.runMode = RunMode.ContinuousRotation;
            return this;
        }

        @Override
        public MotionProfilerRunMode setMotionProfilerRunMode() {
            this.servo.runMode = RunMode.MotionProfiler;
            return this;
        }

        @Override
        public Builder setStandardRunMode() {
            this.servo.runMode = RunMode.Standard;
            return this;
        }

        @Override
        public Builder setPIDRunMode() {
            this.servo.runMode = RunMode.PIDAngle;
            this.servo.feedForwardType = FeedForwardType.Lift;
            return this;
        }

        @Override
        public Builder setMotionProfilerCoefficients(double maxVelocity, double acceleration, double deceleration) {
            servo.setMotionProfilerCoefficients(maxVelocity,acceleration,deceleration);
            return this;
        }
        public Builder setEncoder(HighEncoder encoder){
            servo.encoder = encoder;
            return this;
        }
        public Builder setEncoderResolution(double encoderResolution){
            servo.setEncoderResolution(encoderResolution);
            return this;
        }
        public Builder setPIDCoefficients(double kP , double kI , double kD , double kF) {
            servo.pidfController.setPIDF(kP,kI,kD,kF);
            return this;
        }

        @Override
        public Builder setAnalogInput(AnalogInput analogInput) {
            servo.analogInput = analogInput;
            servo.useAnalogInput = true;
            return this;
        }

        @Override
        public Builder setAnalogInputCoefficients(double error, double minVoltage, double maxVoltage, double minPosition, double maxPosition) {
            servo.setAnalogInputCoefficients(error, minVoltage, maxVoltage, minPosition, maxPosition);
            return this;
        }
        public Builder setInitPosition(double position , boolean isAuto){
            servo.setInitialPosition(position,isAuto);
            return this;
        }
        public HighServo build(){
            return servo;
        }
    }
}