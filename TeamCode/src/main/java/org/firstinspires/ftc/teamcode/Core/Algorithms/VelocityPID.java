package org.firstinspires.ftc.teamcode.Core.Algorithms;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.function.Function;

public class VelocityPID {

    private double kP, kI, kD, kF , kS , kA;

    private double setPoint;
    private double measuredValue;
    private double minOutput, maxOutput;

    private double errorVal_p;
    private double prevErrorVal;
    private double prevPrevErrorVal;
    private double prevOutput;
    private double prevSetPoint;

    private double errorTolerance_p = 0.05;
    private double errorTolerance_v = Double.POSITIVE_INFINITY;

    private final ElapsedTime timer;
    private double lastTimeStampSeconds = -1.0;
    private double periodSeconds = 0.0;

    private double filterGain = 0.8;
    private double lastFilterEstimate = 0.0;
    private double currentFilterEstimate = 0.0;
    private Function<Double,Double> gainScheduler = error -> error;

    /**
     * Base constructor. Initializes with gains, zero setpoint, and zero measured value.
     * @param kp Proportional gain.
     * @param ki Integral gain.
     * @param kd Derivative gain.
     * @param kf Feedforward gain (applied to setpoint changes).
     */
    public VelocityPID(double kp, double ki, double kd, double kf,double kS, double kA, ElapsedTime timer) {
        this(kp, ki, kd, kf,kS,kA ,0, 0, timer);
    }

    /**
     * Full constructor.
     * @param kp Proportional gain.
     * @param ki Integral gain.
     * @param kd Derivative gain.
     * @param kf Feedforward gain (applied to setpoint changes).
     * @param sp Initial setpoint (target velocity).
     * @param pv Initial measured value (current velocity).
     * @param timer ElapsedTime object for calculating time differences.
     */
    public VelocityPID(double kp, double ki, double kd, double kf,double kS, double kA, double sp, double pv, ElapsedTime timer) {
        kP = kp;
        kI = ki;
        kD = kd;
        kF = kf;
        this.timer = timer;

        setPoint = sp;
        measuredValue = pv;

        minOutput = -1.0;
        maxOutput = 1.0;

        reset();
        errorVal_p = setPoint - measuredValue;
    }

    /**
     * Resets the internal state of the controller.
     * Call this when disabling the controller, changing gains significantly, or needing a fresh start.
     */
    public void reset() {
        prevErrorVal = 0;
        prevPrevErrorVal = 0;
        prevOutput = 0;
        lastTimeStampSeconds = -1.0;
        periodSeconds = 0;
        lastFilterEstimate = 0.0;
        currentFilterEstimate = 0.0;

        prevSetPoint = setPoint;

    }

    /**
     * Sets the velocity error tolerance for use with {@link #atSetPoint()}.
     * @param positionTolerance Velocity error which is tolerable (units must match setpoint/measuredValue).
     */
    public void setTolerance(double positionTolerance) {
        setTolerance(positionTolerance, Double.POSITIVE_INFINITY);
    }

    /**
     * Sets the velocity and acceleration error tolerances for use with {@link #atSetPoint()}.
     * @param positionTolerance Velocity error which is tolerable (units must match setpoint/measuredValue).
     * @param velocityTolerance Acceleration error which is tolerable (units/sec).
     */
    public void setTolerance(double positionTolerance, double velocityTolerance) {
        errorTolerance_p = Math.abs(positionTolerance);
        errorTolerance_v = Math.abs(velocityTolerance);
    }

    /**
     * Returns the current setpoint (target velocity).
     * @return The current setpoint.
     */
    public double getSetPoint() {
        return setPoint;
    }

    /**
     * Sets the setpoint (target velocity) for the controller.
     * @param sp The desired setpoint.
     */
    public void setSetPoint(double sp) {
        prevSetPoint = setPoint;
        setPoint = sp;
    }

    /**
     * Returns true if the absolute error and absolute velocity error are within the tolerances
     * set by {@link #setTolerance}.
     * @return Whether the error is within the acceptable bounds.
     */
    public boolean atSetPoint() {
        return Math.abs(getVelocityError()) <= errorTolerance_v;
    }

    /**
     * Returns the PIDF coefficients.
     * @return Array containing {kP, kI, kD, kF}.
     */
    public double[] getCoefficients() {
        return new double[]{kP, kI, kD, kF};
    }

    /**
     * Returns the current velocity error (E_n = setPoint - measuredValue).
     * @return The current velocity error.
     */
    public double getPositionError() {

        return errorVal_p;
    }

    /**
     * Returns the tolerances {position (velocity), velocity (acceleration)} of the controller.
     * @return Array containing {errorTolerance_p, errorTolerance_v}.
     */
    public double[] getTolerance() {
        return new double[]{errorTolerance_p, errorTolerance_v};
    }

    /**
     * Returns the rate of change of the velocity error (approximated acceleration error).
     * e'(t) ≈ (E_n - E_{n-1}) / period.
     * @return The current velocity error rate (acceleration error). Returns 0 if period is too small.
     */
    public double getVelocityError() {
        if (Math.abs(periodSeconds) > 1E-9) {
            return (errorVal_p - prevErrorVal) / periodSeconds;
        }
        return 0;
    }

    /**
     * Calculates the next output of the PIDF controller using the last measured value.
     * Call {@link #calculate(double)} or {@link #calculate(double, double)} to update the measured value first.
     * @return The calculated control output (e.g., motor power).
     */
    public double calculate() {
        return calculate(measuredValue);
    }


    public double calculate(double pv) {
        return calculate(pv,12);
    }
    /**
     * Calculates the control value using the velocity PIDF algorithm with EMA filtering and anti-windup.
     * @param pv The current measurement of the process variable (current velocity).
     * @return The calculated control output, clamped between output bounds.
     */
    public double calculate(double pv, double voltage) {

        double currentTimeStamp = timer.seconds();
        if (lastTimeStampSeconds < 0) {
            lastTimeStampSeconds = currentTimeStamp;
            measuredValue = pv;
            errorVal_p = setPoint - measuredValue;
            prevErrorVal = errorVal_p;
            prevPrevErrorVal = errorVal_p;
            prevSetPoint = setPoint;
            prevOutput = Range.clip(kF * setPoint, minOutput, maxOutput);
            return prevOutput;
        }
        periodSeconds = currentTimeStamp - lastTimeStampSeconds;
        lastTimeStampSeconds = currentTimeStamp;

        prevPrevErrorVal = prevErrorVal;
        prevErrorVal = errorVal_p;
        measuredValue = pv;
        errorVal_p = setPoint - measuredValue;

        if (Math.abs(periodSeconds) < 1E-9) {
            return prevOutput;
        }

        double deltaError = errorVal_p - prevErrorVal;
        double proportionalTerm = kP * gainScheduler.apply(deltaError);

        double integralTerm = kI * errorVal_p * periodSeconds;

        double deltaErrorDerivative = (errorVal_p - 2 * prevErrorVal + prevPrevErrorVal);
        double rawDerivative = deltaErrorDerivative / periodSeconds;

        currentFilterEstimate = (filterGain * lastFilterEstimate) + (1 - filterGain) * rawDerivative;
        lastFilterEstimate = currentFilterEstimate;
        double derivativeTerm = kD * currentFilterEstimate;

        double deltaSetPoint = setPoint - prevSetPoint;
        double staticTerm = kS * Math.signum(deltaSetPoint);
        double accTerm = kA * (deltaSetPoint/periodSeconds);
        double stableTerm = kF * deltaSetPoint;
        double feedForwardChange = stableTerm+staticTerm+accTerm;


        double deltaOutput = proportionalTerm + integralTerm + derivativeTerm + feedForwardChange;

        double currentOutput = prevOutput + deltaOutput;

        if (currentOutput >= maxOutput && integralTerm > 0) {

            integralTerm = Math.max(0, maxOutput - (prevOutput + proportionalTerm + derivativeTerm + feedForwardChange));
            deltaOutput = proportionalTerm + integralTerm + derivativeTerm + feedForwardChange;
            currentOutput = prevOutput + deltaOutput;

        } else if (currentOutput <= minOutput && integralTerm < 0) {

            integralTerm = Math.min(0, minOutput - (prevOutput + proportionalTerm + derivativeTerm + feedForwardChange));
            deltaOutput = proportionalTerm + integralTerm + derivativeTerm + feedForwardChange;
            currentOutput = prevOutput + deltaOutput;
        }
        if(setPoint==0 && closeToNull(prevOutput,pv)){currentOutput = 0;}
        currentOutput = Range.clip(currentOutput, minOutput, maxOutput);
        currentOutput = voltage>=13?currentOutput*(12/voltage):currentOutput;
        currentOutput = Range.clip(currentOutput, minOutput, maxOutput);
        prevOutput = currentOutput;

        return currentOutput;
    }

    /** Sets the PIDF gains. */
    public void setPIDF(double kp, double ki, double kd, double kf) {
        kP = kp;
        kI = ki;
        kD = kd;
        kF = kf;
        kS = 0;
        kA = 0;
    }
    public void setPIDFSA(double kp, double ki, double kd, double kf,double ks, double ka) {
        kP = kp;
        kI = ki;
        kD = kd;
        kF = kf;
        kS = ks;
        kA = ka;
    }

    /**
     * Sets the minimum and maximum output bounds for the controller.
     * @param min The minimum allowable output.
     * @param max The maximum allowable output.
     */
    public void setOutputBounds(double min, double max) {
        if (min >= max) throw new IllegalArgumentException("Minimum output limit cannot be greater than or equal to maximum limit");
        minOutput = min;
        maxOutput = max;

        prevOutput = Range.clip(prevOutput, minOutput, maxOutput);
    }

    /**
     * Renamed from setIntegrationBounds in the provided VelocityPID for clarity.
     * @deprecated Use {@link #setOutputBounds(double, double)} instead.
     */
    @Deprecated
    public void setIntegrationBounds(double min, double max) {
        setOutputBounds(min, max);
    }

    /**
     * Clears the accumulated output (integral state) of the velocity PID controller.
     * Sets the internal previous output (which holds the integral) to zero.
     */
    public void clearTotalError() {
        prevOutput = 0;

    }

    /** Sets the gain for the derivative term's EMA filter (0 <= gain < 1). Lower values filter more. */
    public void setDerivativeFilterGain(double gain) {
        if (gain < 0 || gain >= 1) throw new IllegalArgumentException("Filter gain must be between 0 (inclusive) and 1 (exclusive)");
        filterGain = gain;
    }
    private boolean closeToNull(double po , double cm){
        boolean positionalCondition = Math.abs(cm) < 35;
        boolean previousOutputCondition = Math.abs(po) < 0.15;
        return positionalCondition && previousOutputCondition;
    }
    public void setP(double kp) { kP = kp; }
    public void setI(double ki) { kI = ki; }
    public void setD(double kd) { kD = kd; }
    public void setF(double kf) { kF = kf; }

    public double getP() { return kP; }
    public double getI() { return kI; }
    public double getD() { return kD; }
    public double getF() { return kF; }

    /** Returns the time period (in seconds) between the last two updates. */
    public double getPeriod() { return periodSeconds; }
}