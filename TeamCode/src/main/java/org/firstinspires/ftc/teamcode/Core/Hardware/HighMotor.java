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

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Core.Algorithms.SquidController;
import org.firstinspires.ftc.teamcode.Core.Algorithms.VelocityPID;

public class HighMotor {

    public enum RunMode {
        Squid,
        PID,
        Standard,
        Time,
        Velocity
    }

    public enum FeedForwardType{
        Arm, /// if we want to use feedforward for an arm
        Lift  ///if we want to use feedforward for a lift
    }

    ElapsedTime timer = new ElapsedTime();
    public RunMode runMode;
    public FeedForwardType feedForwardType;
    public DcMotorEx motor;
    public final PIDFController pidfController = new PIDFController(0,0,0,0);
    public final SquidController squidController = new SquidController(0,0,0,0);
    public final VelocityPID pidfVelocity = new VelocityPID(0,0,0,0, timer);
    private double lastPower = -2, power;
    private double multiplier = 1.0;
    private final double epsilon = 1e-5;
    private double currentDrawn = 0;
    private double minimumPowerToOvercomeFriction = 0.0;
    private boolean reverseMotor, reverseEncoder = false;
    private int reverseEncoderMultiplier = 1;
    private boolean useEncoder = false, useZeroPowerBehaviour = true;
    private double target = 0, tolerance = epsilon;
    private double currentPosition = 0, maxPIDPower = 1, kF = 0, initialAngle = 0, ticksPerDegree = 0;
    private double currentVelocity, encoderResolution = 1, wheelDiameter = 0;
    private double time=-1;
    public HighMotor(){}
    /**
     * This constructor can be used for any runMode.
     * It helps us set if the motor is reversed or not.
     * We assume that we don't use an encoder and that we want our motors on brake.
     *
     * @param motor  gives the motor we will work with
     * @param runMode gives the runMode
     * @param reverseMotor is 0 if we don't want to reverse it or 1 if we want to reverse it,
     *                     must use with setReverseMotor()
     */
    public HighMotor(DcMotorEx motor, RunMode runMode, boolean reverseMotor){
        this.motor = motor;
        this.runMode = runMode;
        this.reverseMotor = reverseMotor;

        this.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        setReverseMotor(reverseMotor);
    }

    /**
     *
     * This constructor can be used for any runMode.
     * It helps us - set if the motor is reversed or not
     *             - set if we use zero power behaviour or not (break or float)
     * We assume that we don't use an encoder.
     *
     * @param motor gives the motor we will work with
     * @param runMode gives the runMode
     * @param reverseMotor is 0 if we don't want to reverse it or 1 if we want to reverse it,
     *      *              we set it using the method setReverseMotor()
     * @param useZeroPowerBehaviour is 0 if we want to put our motors on float or 1 if we want to use them on brake,
     *                              we set it using the method setUseZeroPowerBehaviour()
     */
    public HighMotor(DcMotorEx motor, RunMode runMode, boolean reverseMotor, boolean useZeroPowerBehaviour){
        this.motor = motor;
        this.runMode = runMode;
        this.reverseMotor = reverseMotor;

        this.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setUseZeroPowerBehaviour(useZeroPowerBehaviour);
        setReverseMotor(reverseMotor);
    }

    /**
     * This constructor can be used for any runMode.
     * It helps us - set if the motor is reversed or not,
     *             - set if we want to use an encoder or not
     *             - set if we want to reverse the encoder,if we use one
     * We put our motors on brake.
     * Also if the runMode is PID or Squid we make sure that the initial target and tolerance are 0.
     *
     * @param motor gives the motor we will work with
     * @param runMode gives the runMode
     * @param reverseMotor is 0 if we don't want to reverse it or 1 if we want to reverse it,
     *                     we set it using the method setReverseMotor()
     * @param useEncoder is 0 if we don't want to use it or 1 if we want to use it,
     *                   we set it using the method setUseEncoder()
     * @param reverseEncoder is 0 if we don't want to reverse it or 1 if we want to reverse it,
     *                       we set it using the method setReverseEncoder()
     */
    public HighMotor(DcMotorEx motor, RunMode runMode, boolean reverseMotor, boolean useEncoder, boolean reverseEncoder){
        this.motor = motor;
        this.runMode = runMode;
        this.reverseMotor = reverseMotor;
        this.useEncoder = useEncoder;

        if(runMode == RunMode.PID || runMode == RunMode.Squid){
            setTarget(0);
            setTolerance(0);
        }

        this.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        setReverseMotor(reverseMotor);
        setUseEncoder(useEncoder);
        setReverseEncoder(reverseEncoder);
    }

    /**
     * This constructor can be used for any runMode.
     * It helps us - set if the motor is reversed or not,
     *             - set if we use zero power behaviour or not (break or float)
     *             - set if we want to use an encoder or not
     *             - set if we want to reverse the encoder,if we use one
     * We put our motors on brake.
     * Also if the runMode is PID or Squid we make sure that the initial target and tolerance are 0.
     *
     * @param motor gives the motor we will work with
     * @param runMode gives the runMode
     * @param reverseMotor is 0 if we don't want to reverse it or 1 if we want to reverse it,
     *                     we set it using the method setReverseMotor()
     * @param useZeroPowerBehaviour is 0 if we want to put our motors on float or 1 if we want to use them on brake,
     *                              we set it using the method setUseZeroPowerBehaviour()
     * @param useEncoder is 0 if we don't want to use it or 1 if we want to use it,
     *                   we set it using the method setUseEncoder()
     * @param reverseEncoder is 0 if we don't want to reverse it or 1 if we want to reverse it,
     *                       we set it using the method setReverseEncoder()
     */
    public HighMotor(DcMotorEx motor, RunMode runMode, boolean reverseMotor, boolean useZeroPowerBehaviour, boolean useEncoder, boolean reverseEncoder){
        this.motor = motor;
        this.runMode = runMode;
        this.reverseMotor = reverseMotor;
        this.useEncoder = useEncoder;

        if(runMode == RunMode.PID || runMode == RunMode.Squid){
            setTarget(0);
            setTolerance(0);
        }

        this.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setUseZeroPowerBehaviour(useZeroPowerBehaviour);
        setReverseMotor(reverseMotor);
        setUseEncoder(useEncoder);
        setReverseEncoder(reverseEncoder);
    }

    /**
     * wow
     * @param runMode yes
     */
    public void setRunMode(RunMode runMode){
        this.runMode = runMode;
    }

    /**
     * wow
     */
    public RunMode getRunMode(){
        return runMode;
    }

    /**
     * This method resets the motor
     */
    public void resetMotor(){
        this.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * @return this method returns if the motor is reversed or not( 1 if it's reversed or 0 if not)
     */
    public boolean isReverseMotor() {
        return reverseMotor;
    }

    /**
     * This method helps us set if the motor is reversed or not.
     * This method is called in the constructors.
     *
     * @param reverseMotor is 0 if we don't want to reverse it or 1 if we want to reverse it
     */
    public void setReverseMotor(boolean reverseMotor) {
        this.reverseMotor = reverseMotor;
        if(reverseMotor){
            this.motor.setDirection(DcMotor.Direction.REVERSE);
        } else {
            this.motor.setDirection(DcMotor.Direction.FORWARD);
        }
    }

    /**
     *
     * @return this method returns the encoder multiplier( 1 if the encoder isn't reversed or -1 if it is reversed)
     */
    public int getReverseEncoderMultiplier() {
        return reverseEncoderMultiplier;
    }

    /**
     * This method reverses the encoder if wanted to by setting the multiplier of the encoder.
     * ( 1 if the encoder isn't reversed or -1 if it is reversed)
     * Everytime we use the output of the encoder we multiply by this multiplier.
     *
     * @param reverseEncoder is 0 if we don't want to reverse it or 1 if we want to reverse it
     */
    public void setReverseEncoder(boolean reverseEncoder) {
        this.reverseEncoder = reverseEncoder;
        if(reverseEncoder){
            reverseEncoderMultiplier = -1;
        } else {
            reverseEncoderMultiplier = 1;
        }
    }

    /**
     *
     * @return this method returns if we use zero power behaviour (1 if we want our motors on break
     * or 0 if we want our motors on float)
     */
    public boolean getUseZeroPowerBehaviour() {
        return useZeroPowerBehaviour;
    }

    /**
     * This method helps us set our motors on brake of float, based on the value of useZeroPowerBehaviour
     *
     * @param useZeroPowerBehaviour is 0 if we want to put our motors on float or 1 if we want to use them on brake
     */
    public void setUseZeroPowerBehaviour(boolean useZeroPowerBehaviour) {
        this.useZeroPowerBehaviour = useZeroPowerBehaviour;
        if(useZeroPowerBehaviour){
            this.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } else {
            this.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }


    /**
     * This method helps us set if we use encoder or not, based on the value of useEncoder.
     *
     * @param useEncoder is 0 if we don't want to use it or 1 if we want to use it
     */
    public void setUseEncoder(boolean useEncoder) {
        this.useEncoder = useEncoder;
        if(useEncoder){
            this.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            this.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    /**
     * This method adjusts the raw power value to compensate for friction, ensuring that the mechanism
     * receives enough power to start moving even when friction would resist small power inputs.*
     * ! This method must be used with setMinimumPowerToOvercomeFriction()
     * The given power is first scaled using the wanted multiplier and its clamped to the range [-1,1]
     * to ensure it's within the acceptable limits.
     * Then we calculate coefficientPowerDistribution by adjusting the minimum power needed based on
     * the current system voltage.
     * The remaining power is scaled by (1 - coefficientPowerDistribution) to ensure the total remains within bounds,
     * and the friction compensation multiplied with the power's sign is added to the power
     *
     * @param power gives the raw power that needs to be adjusted
     */
    public void setPowerToOvercomeFriction(double power) {
        power = power * multiplier;
        power = Range.clip(power, -1.0, 1.0);
        double coefficientPowerDistribution = minimumPowerToOvercomeFriction * (12.0 / Constants.Globals.voltage );
        power *= (1 - coefficientPowerDistribution);
        this.power = power + coefficientPowerDistribution * Math.signum(power);
    }

    /**
     * This method sets the variable minimumPowerToOvercomeFriction we will use in the method setPowerToOvercomeFriction()
     *
     * @param minimumPowerToOvercomeFriction gives the value of the minimum power to overcome static friction
     */
    public void setMinimumPowerToOvercomeFriction(double minimumPowerToOvercomeFriction){
        this.minimumPowerToOvercomeFriction = minimumPowerToOvercomeFriction;
    }

    /**
     *
     * @return this method return the minimum power to overcome static friction
     */
    public double getMinimumPowerToOvercomeFriction(){
        return minimumPowerToOvercomeFriction;
    }

    /**
     * This method sets the multiplier that will help us scale the power.
     *
     * @param multiplier gives the value of the multiplier
     */
    public void setMultiplier(double multiplier){
        this.multiplier = multiplier;
    }

    /**
     *
     * @return this method returns the value of the multiplier
     */
    public double getMultiplier(){
        return multiplier;
    }

    /**
     * This method adjusts the raw power value and can be used if the runMode selected is Squid, PID or Standard.
     * The given power is first scaled using the wanted multiplier and its clamped to the range [-1,1]
     * to ensure it's within the acceptable limits.
     *
     * @param power gives the raw power that needs to be adjusted
     */
    public void setPower(double power){
        power = power * multiplier;
        this.power = Range.clip(power, -1.0, 1.0);
        time = -1;
    }

    /**
     * This method adjusts the raw power value and can be used if the runMode selected is Time.
     * The given power is first scaled using the wanted multiplier and its clamped to the range [-1,1]
     * to ensure it's within the acceptable limits.
     * It also sets the time we should run the mechanism and resets the timer.
     *
     * @param power gives the raw power that needs to be adjusted
     * @param time gives the duration for which the power should be applied
     */
    public void setPower(double power, double time){
        power = power * multiplier;
        this.time = time;
        this.power = Range.clip(power, -1.0, 1.0);
        timer.reset();
    }

    /**
     *
     * @return this method returns the power
     */
    public double getPower() {
        return power;
    }

    /**
     *
     * @return this method returns the current drawn by the motor
     * This value is updated in the update() method.
     */
    public double getCurrentDrawn() {
        return currentDrawn;
    }

    /**
     *
     * @return this method returns the velocity of the motor
     */
    public double getVelocity() {
//        if(runMode.equals(RunMode.Velocity)) {
//            return (motor.getVelocity() / encoderResolution) * wheelDiameter;
//        }else {
//            return motor.getVelocity();
//        }
        return motor.getVelocity();
    }

    // PID and Squid stuff

    /**
     *
     * @return this method returns the target
     */
    public double getTarget() {
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
        squidController.setSetPoint(target);
        pidfVelocity.setSetPoint((target/wheelDiameter)*encoderResolution);
    }

    /**
     *
     * @return this method returns true if the motor is at the desired position or false if not*
     * We return true if the absolute value of the difference between the wanted target and the current
     * position is smaller then the accepted error (tolerance) or false if not.
     */
    public boolean atTarget() {
        return Math.abs(target - currentPosition) <= tolerance;
    }

    /**
     *
     * @return this method returns the tolerance
     */
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
        squidController.setTolerance(tolerance);
        pidfVelocity.setTolerance(tolerance);
    }

    /**
     *
     * @return this method returns the current position
     */
    public double getCurrentPosition(){
        return currentPosition;
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
        double PidPower = pidfController.calculate(currentPosition, target);
        double FeedForwardArm = Math.cos(Math.toRadians(initialAngle + target / ticksPerDegree))*kF;
        double FeedForwardLift = target * kF;
        double FeedForward = 0;
        switch (feedForwardType){
            case Arm:
                FeedForward = FeedForwardArm;
                break;
            case Lift:
                FeedForward = FeedForwardLift;
                break;
        }

        return Range.clip(PidPower + FeedForward, -maxPIDPower, maxPIDPower);
    }

    //Squid stuff:

    /**
     * This method sets the Squid coefficients (kP, kI, kD) for the Squid controller.
     * This method is used when we only want to configure the basic proportional,
     * integral, and derivative gains, and no feedforward (kF = 0).
     *
     * @param kP the proportional gain
     * @param kI the integral gain
     * @param kD the derivative gain
     */
    public void setSquidCoefficients(double kP, double kI, double kD) {
        squidController.setPIDF(kP, kI, kD, 0);
    }

    /**
     * This method sets the Squid coefficients (kP, kI, kD) for the Squid controller.
     * This method is used when we only want to configure the basic proportional,
     * integral, and derivative gains, and no feedforward (kF = 0).
     * Also it sets the maximum power that the Squid controller is allowed to apply.
     *
     * @param kP the proportional gain
     * @param kI the integral gain
     * @param kD the derivative gain
     * @param maxPIDPower the value of the maximum power our Squid controller is allowed to apply
     */
    public void setSquidCoefficients(double kP, double kI, double kD, double maxPIDPower) {
        this.maxPIDPower = Math.abs(maxPIDPower);
        squidController.setPIDF(kP, kI, kD, 0);
    }

    /**
     * Sets the Squid coefficients (kP, kI, kD) along with feedforward gain (kF), the feed forward type(Arm or Lift),
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
    public void setSquidCoefficients(double kP, double kI, double kD, double kF, FeedForwardType feedForwardType, double initialAngle, double ticksPerDegree) {
        this.kF = kF;
        this.feedForwardType = feedForwardType;
        this.initialAngle = initialAngle;
        this.ticksPerDegree = ticksPerDegree;
        squidController.setPIDF(kP, kI, kD, 0);
    }

    /**
     * Sets the Squid coefficients (kP, kI, kD) along with feedforward gain (kF), the feed forward type(Arm or Lift),
     * initial angle, ticks per degree and the maximum power our Squid controller is allowed to apply
     *
     *
     * @param kP the proportional gain
     * @param kI the integral gain
     * @param kD the derivative gain
     * @param kF the feedforward gain
     * @param feedForwardType the feedforward type(Arm or Lift)
     * @param initialAngle the initial angle
     * @param ticksPerDegree gives how many encoder ticks correspond to one degree of movement
     * @param maxPIDPower the value of the maximum power our Squid controller is allowed to apply
     */
    public void setSquidCoefficients(double kP, double kI, double kD, double kF, FeedForwardType feedForwardType, double initialAngle, double ticksPerDegree , double maxPIDPower) {
        this.kF = kF;
        this.feedForwardType = feedForwardType;
        this.initialAngle = initialAngle;
        this.ticksPerDegree = ticksPerDegree;
        this.maxPIDPower = Math.abs(maxPIDPower);
        squidController.setPIDF(kP, kI, kD, 0);

    }

    /**
     * This method resets the Squid. This clears any accumulated error.
     */
    public void resetSquid() {
        squidController.reset();
    }

    /**
     *
     * This method calculates the total motor power using a combination of Squid control and feedforward compensation.
     * First of all we calculate the Squid power based on the current position and target. Then we calculate the
     * feedforward depending on the feedforward type (Arm or Lift).
     * We calculate the power needed to hold or move the arm/lift based on its angular position.
     * In the end, the total output( Squid power + FeedForward ) is clipped to the maximum allowed Squid power.
     * @param currentPosition gives the current position
     * @return this method returns the final power to be applied to the motor, in the range [-maxPIDPower, maxPIDPower]
     */
    public double getPowerSquid(double currentPosition) {
        this.currentPosition = currentPosition;
        double SquidPower = squidController.calculate(currentPosition, target);
        double FeedForwardArm = Math.cos(Math.toRadians(initialAngle + target / ticksPerDegree))*kF;
        double FeedForwardLift = target * kF;
        double FeedForward = 0;
        switch (feedForwardType){
            case Arm:
                FeedForward = FeedForwardArm;
                break;
            case Lift:
                FeedForward = FeedForwardLift;
                break;
        }

        return Range.clip(SquidPower + FeedForward, -maxPIDPower, maxPIDPower);
    }

    //Velocity stuff:

    public void setEncoderResolution(double encoderResolution) {
        this.encoderResolution = encoderResolution;
    }

    public double getEncoderResolution(){
        return encoderResolution;
    }
    public double getWheelDiameter(){return wheelDiameter;}


    /**
     * This method sets the Squid coefficients (kP, kI, kD) for the Squid controller.
     * This method is used when we only want to configure the basic proportional,
     * integral, and derivative gains, and no feedforward (kF = 0).
     *
     * @param kP the proportional gain
     * @param kI the integral gain
     * @param kD the derivative gain
     */
    public void setVelocityPIDCoefficients(double kP, double kI, double kD) {
        pidfVelocity.setPIDF(kP, kI, kD, 0);
    }

    /**
     * This method sets the Squid coefficients (kP, kI, kD) for the Squid controller.
     * This method is used when we only want to configure the basic proportional,
     * integral, and derivative gains, and no feedforward (kF = 0).
     * Also it sets the maximum power that the Squid controller is allowed to apply.
     *
     * @param kP the proportional gain
     * @param kI the integral gain
     * @param kD the derivative gain
     * @param maxPIDPower the value of the maximum power our Squid controller is allowed to apply
     */
    public void setVelocityPIDCoefficients(double kP, double kI, double kD, double maxPIDPower) {
        this.maxPIDPower = Math.abs(maxPIDPower);
        pidfVelocity.setPIDF(kP, kI, kD, 0);
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
     * @param maxPIDPower the value of the maximum power our PID controller is allowed to apply
     */
    public void setVelocityPIDCoefficients(double kP, double kI, double kD, double kF, double maxPIDPower) {
        pidfVelocity.setPIDF(kP, kI, kD, kF);
        this.maxPIDPower = Math.abs(maxPIDPower);
    }
    public void setWheelDiameter(double wheelDiameter){
        this.wheelDiameter = wheelDiameter;
    }

    public double getPowerVelocity(double currentVelocity) {
        this.currentVelocity = currentVelocity;
        double VelocityPower = pidfVelocity.calculate(currentVelocity, target);
        return Range.clip(VelocityPower, -maxPIDPower, maxPIDPower);
    }

    /**
     * This method updates the motor control logic depending on the current runMode.
     * If we use an encoder, the current motor position is read and adjusted
     * using the reverse encoder multiplier.
     * Then the motor’s current draw is also updated.
     * If the runMode selected is Squid we calculate power using a Squid controller. Power is updated only if
     * the absolute value of the difference between the current power and the last power is bigger than epsilon.
     * If the runMode selected is PID we calculate power using a PID controller. Power is updated only if
     * the absolute value of the difference between the current power and the last power is bigger than epsilon.
     * If the runMode selected is Standard we apply the raw power directly. Power is updated only if
     * the absolute value of the difference between the current power and the last power is bigger than epsilon.
     * If the runMode selected is Time we apply the power for a period of time .After the timer expires, the motor
     * is stopped automatically. Power is updated only if the absolute value of the difference between
     * the current power and the last power is bigger than epsilon.
     */
    public void update(){
        if(useEncoder){
            currentPosition = motor.getCurrentPosition() * reverseEncoderMultiplier;
        }
        currentDrawn = motor.getCurrent(CurrentUnit.AMPS);
        switch(runMode){
            case Squid:
                power = getPowerSquid(currentPosition);
                if (Math.abs(power - lastPower) >= epsilon) {
                    motor.setPower(power);
                    lastPower = power;
                }
                break;
            case PID:
                power = getPowerPID(currentPosition);
                if (Math.abs(power - lastPower) >= epsilon) {
                    motor.setPower(power);
                    lastPower = power;
                }
                break;
            case Velocity:
                currentVelocity = getVelocity();
                power = pidfVelocity.calculate(currentVelocity);
                if (Math.abs(power - lastPower) >= epsilon) {
                    motor.setPower(power);
                    lastPower = power;
                }
                break;
            case Standard:
                if (Math.abs(power - lastPower) >= epsilon) {
                    motor.setPower(power);
                    lastPower = power;
                }
                break;
            case Time:
                if (Math.abs(power - lastPower) >= epsilon) {
                    motor.setPower(power);
                    lastPower = power;
                }
                if(timer.milliseconds() >= time && power != 0 && time != -1){
                    motor.setPower(0);
                    power = 0;
                }
                break;
        }
    }

    /**
     * This method sends various variables to the Driver Hub to facilitate easier debugging.
     * @param telemetry this is the telemetry that is going to be used
     */
    public void telemetry(Telemetry telemetry){
        telemetry.addData("Run mode: ", runMode);
        telemetry.addData("Last power: ", lastPower);
        telemetry.addData("Power: ", getPower());
        telemetry.addData("Motor velocity: ", getVelocity());
        telemetry.addData("Current drawn: ", getCurrentDrawn());
        telemetry.addData("Multiplier: ", getMultiplier());
        telemetry.addData("Minimum power to overcome friction: ", getMinimumPowerToOvercomeFriction());
        telemetry.addData("Reverse motor: ", isReverseMotor());
        if(useEncoder){
            telemetry.addData("Reverse encoder: ", reverseEncoder);
            telemetry.addData("Reverse encoder multiplier: ", getReverseEncoderMultiplier());
        }
        telemetry.addData("Zero power behaviour: ", getUseZeroPowerBehaviour());
        switch(runMode){
            case Squid:
                telemetry.addData("Error: ", target - currentPosition);
                telemetry.addData("Target: ", getTarget());
                telemetry.addData("Current position: ", getCurrentPosition());
                telemetry.addData("Power squid: ", getPowerSquid(currentPosition));
                telemetry.addData("Feed forward type: ", getFeedForwardType());
                telemetry.addData("Max PID power: ", maxPIDPower);
                telemetry.addData("kF: ", kF);
                telemetry.addData("Tolerance: ", getTolerance());
                if(feedForwardType == FeedForwardType.Arm){
                    telemetry.addData("Initial angle: ", getInitialAngle());
                    telemetry.addData("Ticks per degree: ", getTicksPerDegree());
                }
                break;
            case PID:
                telemetry.addData("Error: ", target - currentPosition);
                telemetry.addData("Target: ", getTarget());
                telemetry.addData("Current position: ", getCurrentPosition());
                telemetry.addData("Power PID: ", getPowerPID(currentPosition));
                telemetry.addData("Feed forward type: ", getFeedForwardType());
                telemetry.addData("Max PID power: ", maxPIDPower);
                telemetry.addData("kF: ", kF);
                telemetry.addData("Tolerance: ", getTolerance());
                if(feedForwardType == FeedForwardType.Arm){
                    telemetry.addData("Initial angle: ", getInitialAngle());
                    telemetry.addData("Ticks per degree: ", getTicksPerDegree());
                }
                break;
            case Velocity:
                telemetry.addData("Error: ", target - currentVelocity);
                telemetry.addData("Target: ", getTarget());
                telemetry.addData("Current Velocity: ", currentVelocity);
                telemetry.addData("Power Velocity: ", getPowerVelocity(currentVelocity));
                telemetry.addData("Max PID power: ", maxPIDPower);
                telemetry.addData("Tolerance: ", getTolerance());
                break;
            case Standard:
                break;
            case Time:
                telemetry.addData("Timer time : ", timer.milliseconds());
                telemetry.addData("Target time: ", time);
                break;
        }
    }
    /*
       |
       | BUILDER CLASS
       |
     */

    public interface MotorStep {

        public RunModeStep setMotor(DcMotorEx motor);
    }

    public interface RunModeStep {

        public ReverseMotorStep setRunMode(RunMode runMode);
    }

    public interface ReverseMotorStep {

        public Builder setReverseMotor(boolean reverse);
    }

    public static class Builder implements
            MotorStep,
            RunModeStep,
            ReverseMotorStep {

        private HighMotor motor = new HighMotor();

        private Builder() {
        }

        public static MotorStep startBuilding() {
            return new Builder();
        }

        @Override
        public RunModeStep setMotor(DcMotorEx motor) {
            this.motor.motor = motor;

            this.motor.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            this.motor.setUseZeroPowerBehaviour(true);
            return this;
        }

        @Override
        public ReverseMotorStep setRunMode(RunMode runMode) {
            this.motor.runMode = runMode;

            if (runMode == RunMode.PID || runMode == RunMode.Squid) {
                this.motor.setTarget(0);
                this.motor.setTolerance(0);
            }
            return this;
        }

        @Override
        public Builder setReverseMotor(boolean reverse) {
            this.motor.setReverseMotor(reverse);
            return this;
        }

        public Builder setUseZeroPowerBehaviour(boolean useBrake) {
            this.motor.setUseZeroPowerBehaviour(useBrake);
            return this;
        }

        public Builder setEncoder(boolean useEncoder, boolean reverseEncoder) {
            this.motor.setUseEncoder(useEncoder);
            this.motor.setReverseEncoder(reverseEncoder);
            return this;
        }

        public Builder setMultiplier(double multiplier) {
            this.motor.setMultiplier(multiplier);
            return this;
        }

        public Builder setMinimumPowerToOvercomeFriction(double minPower) {
            this.motor.setMinimumPowerToOvercomeFriction(minPower);
            return this;
        }

        public Builder setPIDCoefficients(double kP, double kI, double kD) {
            motor.setPIDCoefficients(kP, kI, kD);
            return this;
        }

        public Builder setPIDCoefficients(double kP, double kI, double kD, double maxPIDPower) {
            motor.setPIDCoefficients(kP, kI, kD, maxPIDPower);
            return this;
        }

        public Builder setPIDCoefficients(double kP, double kI, double kD, double kF, FeedForwardType feedForwardType, double initialAngle, double ticksPerDegree) {
            motor.setPIDCoefficients(kP, kI, kD, kF, feedForwardType, initialAngle, ticksPerDegree);
            return this;
        }

        public Builder setPIDCoefficients(double kP, double kI, double kD, double kF, FeedForwardType feedForwardType) {
            motor.setPIDCoefficients(kP, kI, kD, kF, feedForwardType);
            return this;
        }

        public Builder setPIDCoefficients(double kP, double kI, double kD, double kF, FeedForwardType feedForwardType, double maxPIDPower) {
            motor.setPIDCoefficients(kP, kI, kD, kF, feedForwardType, maxPIDPower);
            return this;
        }

        public Builder setPIDCoefficients(double kP, double kI, double kD, double kF, FeedForwardType feedForwardType, double initialAngle, double ticksPerDegree, double maxPIDPower) {
            motor.setPIDCoefficients(kP, kI, kD, kF, feedForwardType, initialAngle, ticksPerDegree, maxPIDPower);
            return this;
        }

        public Builder setSquidCoefficients(double kP, double kI, double kD) {
            motor.setSquidCoefficients(kP, kI, kD);
            return this;
        }

        public Builder setSquidCoefficients(double kP, double kI, double kD, double maxPIDPower) {
            motor.setSquidCoefficients(kP, kI, kD, maxPIDPower);
            return this;
        }

        public Builder setSquidCoefficients(double kP, double kI, double kD, double kF, FeedForwardType feedForwardType, double initialAngle, double ticksPerDegree) {
            motor.setSquidCoefficients(kP, kI, kD, kF, feedForwardType, initialAngle, ticksPerDegree);
            return this;
        }

        public Builder setSquidCoefficients(double kP, double kI, double kD, double kF, FeedForwardType feedForwardType, double initialAngle, double ticksPerDegree, double maxPIDPower) {
            motor.setSquidCoefficients(kP, kI, kD, kF, feedForwardType, initialAngle, ticksPerDegree, maxPIDPower);
            return this;
        }

        public Builder setVelocityPIDCoefficients(double kP, double kI, double kD) {
            motor.setVelocityPIDCoefficients(kP, kI, kD);
            return this;
        }

        public Builder setVelocityPIDCoefficients(double kP, double kI, double kD, double maxPIDPower) {
            motor.setVelocityPIDCoefficients(kP, kI, kD, maxPIDPower);
            return this;
        }

        public Builder setVelocityPIDCoefficients(double kP, double kI, double kD, double kF, double maxPIDPower) {
            motor.setVelocityPIDCoefficients(kP, kI, kD, kF, maxPIDPower);
            return this;
        }

        public Builder setEncoderResolution(double encoderResolution) {
            motor.setEncoderResolution(encoderResolution);
            return this;
        }

        public Builder setWheelDiameter(double wheelDiameter) {
            motor.setWheelDiameter(wheelDiameter);
            return this;
        }

        public HighMotor build() {

            return this.motor;
        }
    }
}