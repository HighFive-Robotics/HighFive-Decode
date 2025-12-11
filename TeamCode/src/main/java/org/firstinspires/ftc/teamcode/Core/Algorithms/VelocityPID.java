package org.firstinspires.ftc.teamcode.Core.Algorithms;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.function.Function;

public class VelocityPID {

    private double kP, kI, kD, kF, kS, kA;

    private double setPoint;
    private double measuredValue;
    private double minOutput, maxOutput;

    private double errorVal_p;
    private double prevErrorVal;
    private double prevPrevErrorVal;

    private double prevBaseOutput;
    private double prevSetPoint;

    private double errorTolerance_p = 0.05;
    private double errorTolerance_v = Double.POSITIVE_INFINITY;

    private final ElapsedTime timer;
    private double lastTimeStampSeconds = -1.0;
    private double periodSeconds = 0.0;

    private double filterGain = 0.8;
    private double lastFilterEstimate = 0.0;
    private double currentFilterEstimate = 0.0;

    private Function<Double, Double> gainScheduler = error -> error;

    public VelocityPID(double kp, double ki, double kd, double kf, double kS, double kA, ElapsedTime timer) {
        this(kp, ki, kd, kf, kS, kA, 0, 0, timer);
    }

    public VelocityPID(double kp, double ki, double kd, double kf, double kS, double kA, double sp, double pv, ElapsedTime timer) {
        kP = kp;
        kI = ki;
        kD = kd;
        kF = kf;
        this.kS = kS;
        this.kA = kA;
        this.timer = timer;

        setPoint = sp;
        measuredValue = pv;

        minOutput = -1.0;
        maxOutput = 1.0;

        reset();
        errorVal_p = setPoint - measuredValue;
    }

    public void reset() {
        prevErrorVal = 0;
        prevPrevErrorVal = 0;
        prevBaseOutput = 0;

        lastTimeStampSeconds = -1.0;
        periodSeconds = 0;
        lastFilterEstimate = 0.0;
        currentFilterEstimate = 0.0;

        prevSetPoint = setPoint;
    }

    public void setTolerance(double positionTolerance) {
        setTolerance(positionTolerance, Double.POSITIVE_INFINITY);
    }

    public void setTolerance(double positionTolerance, double velocityTolerance) {
        errorTolerance_p = Math.abs(positionTolerance);
        errorTolerance_v = Math.abs(velocityTolerance);
    }

    public double getSetPoint() {
        return setPoint;
    }

    public void setSetPoint(double sp) {
        prevSetPoint = setPoint;
        setPoint = sp;
    }

    public boolean atSetPoint() {
        return Math.abs(getVelocityError()) <= errorTolerance_v;

    }

    public double[] getCoefficients() {
        return new double[]{kP, kI, kD, kF};
    }

    public double getPositionError() {
        return errorVal_p;
    }

    public double[] getTolerance() {
        return new double[]{errorTolerance_p, errorTolerance_v};
    }

    public double getVelocityError() {
        if (Math.abs(periodSeconds) > 1E-9) {
            return (errorVal_p - prevErrorVal) / periodSeconds;
        }
        return 0;
    }

    public double calculate() {
        return calculate(measuredValue);
    }

    public double calculate(double pv) {
        return calculate(pv, 12);

    }

    public double calculate(double pv, double voltage) {
        double currentTimeStamp = timer.seconds();

        if (lastTimeStampSeconds < 0) {
            lastTimeStampSeconds = currentTimeStamp;
            measuredValue = pv;
            errorVal_p = setPoint - measuredValue;
            prevErrorVal = errorVal_p;
            prevPrevErrorVal = errorVal_p;
            prevSetPoint = setPoint;

            prevBaseOutput = Range.clip(kF * setPoint, minOutput, maxOutput);
            return prevBaseOutput;
        }

        periodSeconds = currentTimeStamp - lastTimeStampSeconds;
        lastTimeStampSeconds = currentTimeStamp;

        prevPrevErrorVal = prevErrorVal;
        prevErrorVal = errorVal_p;
        measuredValue = pv;
        errorVal_p = setPoint - measuredValue;

        if (Math.abs(periodSeconds) < 1E-9) {
            return prevBaseOutput;

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
        double velocityFFTerm = kF * deltaSetPoint;

        double deltaBaseOutput = proportionalTerm + integralTerm + derivativeTerm + velocityFFTerm;

        double currentBaseOutput = prevBaseOutput + deltaBaseOutput;

        if (currentBaseOutput >= maxOutput && integralTerm > 0) {
            integralTerm = Math.max(0, maxOutput - (prevBaseOutput + proportionalTerm + derivativeTerm + velocityFFTerm));
            deltaBaseOutput = proportionalTerm + integralTerm + derivativeTerm + velocityFFTerm;
            currentBaseOutput = prevBaseOutput + deltaBaseOutput;
        } else if (currentBaseOutput <= minOutput && integralTerm < 0) {
            integralTerm = Math.min(0, minOutput - (prevBaseOutput + proportionalTerm + derivativeTerm + velocityFFTerm));
            deltaBaseOutput = proportionalTerm + integralTerm + derivativeTerm + velocityFFTerm;
            currentBaseOutput = prevBaseOutput + deltaBaseOutput;
        }

        currentBaseOutput = Range.clip(currentBaseOutput, minOutput, maxOutput);

        prevBaseOutput = currentBaseOutput;

        double staticTerm = 0;
        if (Math.abs(setPoint) > 0.001) {
            staticTerm = kS * Math.signum(setPoint);
        }

        double accTerm = kA * (deltaSetPoint / periodSeconds);

        double finalOutput = currentBaseOutput + staticTerm + accTerm;

        return Range.clip(finalOutput, minOutput, maxOutput);
    }

    public void setPIDF(double kp, double ki, double kd, double kf) {
        kP = kp;
        kI = ki;
        kD = kd;
        kF = kf;
        kS = 0;
        kA = 0;
    }

    public void setPIDFSA(double kp, double ki, double kd, double kf, double ks, double ka) {
        kP = kp;
        kI = ki;
        kD = kd;
        kF = kf;
        kS = ks;
        kA = ka;
    }

    public void setOutputBounds(double min, double max) {
        if (min >= max) throw new IllegalArgumentException("Minimum output limit cannot be greater than or equal to maximum limit");
        minOutput = min;
        maxOutput = max;

        prevBaseOutput = Range.clip(prevBaseOutput, minOutput, maxOutput);
    }

    @Deprecated
    public void setIntegrationBounds(double min, double max) {
        setOutputBounds(min, max);
    }

    public void clearTotalError() {
        prevBaseOutput = 0;
    }

    public void setDerivativeFilterGain(double gain) {
        if (gain < 0 || gain >= 1) throw new IllegalArgumentException("Filter gain must be between 0 (inclusive) and 1 (exclusive)");
        filterGain = gain;
    }

    public void setP(double kp) { kP = kp; }
    public void setI(double ki) { kI = ki; }
    public void setD(double kd) { kD = kd; }
    public void setF(double kf) { kF = kf; }

    public double getP() { return kP; }
    public double getI() { return kI; }
    public double getD() { return kD; }
    public double getF() { return kF; }

    public double getPeriod() { return periodSeconds; }
}