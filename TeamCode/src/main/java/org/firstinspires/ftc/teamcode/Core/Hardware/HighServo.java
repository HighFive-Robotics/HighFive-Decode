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


public class HighServo {

    public enum RunMode {
        MotionProfiler,//Must use the method setMotionProfilerCoefficients();
        Standard,
        ContinuousRotation
    }

    public Servo servo;
    public CRServo CRServo;
    public AnalogInput analogInput;
    private final AsymmetricMotionProfiler motionProfiler = new AsymmetricMotionProfiler(1, 1, 1);
    private double targetPosition, targetPositionMotionProfile, currentPosition, lastPosition = -1;
    private final double epsilon = 1e-5;
    private boolean useAnalogInput = false, atTarget = false;
    private double error = epsilon, voltage, minPosition = 0, maxPosition = 1, minVoltage = 0, maxVoltage = 3.3;
    private double power, lastPower = -2;

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
        switch (runMode) {
            case MotionProfiler:
                lastPosition = targetPosition;
                targetPosition = position;
                targetPositionMotionProfile = position;
                motionProfiler.setMotion(lastPosition, targetPositionMotionProfile);
                break;
            case Standard:
                targetPosition = position;
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
    public double getPosition() {
        return currentPosition;
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
                if (Math.abs(targetPosition - lastPosition) >= epsilon) {
                    servo.setPosition(targetPosition);
                    lastPosition = targetPosition;
                }
                break;
            case ContinuousRotation:
                if (Math.abs(power - lastPower) >= epsilon) {
                    CRServo.setPower(power);
                    lastPower = power;
                }
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
}