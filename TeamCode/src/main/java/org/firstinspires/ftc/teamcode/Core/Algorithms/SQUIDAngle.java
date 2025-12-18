package org.firstinspires.ftc.teamcode.Core.Algorithms;

import java.nio.channels.ClosedByInterruptException;
import java.util.function.Function;

public class SQUIDAngle {
    private double kP, kI, kD, kF;

    private double setPoint;
    private double measuredValue;
    private double minIntegral, maxIntegral;
    private double errorVal_p;
    private double errorVal_v;
    public double totalError;
    private double prevErrorVal;
    private double errorTolerance_p = 0.05;
    private double errorTolerance_v = Double.POSITIVE_INFINITY;
    private double integralZone = 20.0;
    private double lastTimeStamp;
    private double period;
    private double lastDerivative = 0;
    private double derivativeFilterGain = 0.8;
    private final double linThreshold = 900;
    private final double angleMultiplier = 100;

    public Function<Double,Double> gainS = correction -> {
        if(Math.abs(correction) < linThreshold){
            double newCorrection = (correction)*(linThreshold/angleMultiplier);
            return correction;
        }else return correction*2.5;
    };
    public Function<Double,Double> coefGain = coef -> {
        return coef * 1.0;
    };

    public SQUIDAngle(double kp, double ki, double kd, double kf) {
        this(kp, ki, kd, kf, 0, 0);
    }

    public SQUIDAngle(double kp, double ki, double kd, double kf, double sp, double pv) {
        kP = kp;
        kI = ki;
        kD = kd;
        kF = kf;
        setPoint = angleMultiplier*sp;
        setIntegralZone(integralZone);
        measuredValue = pv*angleMultiplier;
        minIntegral = -1.0;
        maxIntegral = 1.0;
        lastTimeStamp = 0;
        period = 0;
        errorVal_p = setPoint - measuredValue;
        reset();
    }

    public void reset() {
        totalError = 0;
        prevErrorVal = 0;
        lastTimeStamp = 0;
        lastDerivative = 0;
    }

    public void setTolerance(double positionTolerance) {
        setTolerance(positionTolerance, Double.POSITIVE_INFINITY);
    }

    public void setTolerance(double positionTolerance, double velocityTolerance) {
        errorTolerance_p = angleMultiplier*positionTolerance;
        errorTolerance_v = angleMultiplier*velocityTolerance;
    }

    public void setIntegralZone(double degrees) {
        this.integralZone = degrees * angleMultiplier;
    }

    public void setDerivativeFilter(double gain) {
        this.derivativeFilterGain = gain;
    }

    public double getSetPoint() {
        return setPoint;
    }

    public void setSetPoint(double sp) {
        setPoint = sp*angleMultiplier;
        errorVal_p = (setPoint - measuredValue) % (360.0*angleMultiplier);
        if (errorVal_p > (180*angleMultiplier)) {
            errorVal_p -= (360.0*angleMultiplier);
        } else if (errorVal_p <(-180*angleMultiplier)) {
            errorVal_p += (360.0*angleMultiplier);
        }
        if (Math.abs(period) > 1E-6) {
            double rawDerivative = (errorVal_p - prevErrorVal) / period;
            errorVal_v = (derivativeFilterGain * lastDerivative) + ((1 - derivativeFilterGain) * rawDerivative);
            lastDerivative = errorVal_v;
        } else {
            errorVal_v = 0;
        }
    }

    public boolean atSetPoint() {
        return Math.abs(errorVal_p) < errorTolerance_p && Math.abs(errorVal_v) < errorTolerance_v;
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
        return errorVal_v;
    }

    public double calculate() {
        return calculate(measuredValue);
    }

    public double calculate(double pv, double sp) {
        setSetPoint(sp);
        return calculate(pv);
    }

    public double calculate(double pv) {
        double currentTimeStamp = System.nanoTime() * 1.0E-9;
        if (lastTimeStamp == 0) lastTimeStamp = currentTimeStamp;
        period = currentTimeStamp - lastTimeStamp;
        lastTimeStamp = currentTimeStamp;
        prevErrorVal = errorVal_p;
        measuredValue = pv*angleMultiplier;
        errorVal_p = (setPoint - measuredValue) % (360.0*angleMultiplier);

        if (errorVal_p > (180.0*angleMultiplier)) {
            errorVal_p -= (360.0*angleMultiplier);
        } else if (errorVal_p < (-180.0*angleMultiplier)) {
            errorVal_p += (360.0*angleMultiplier);
        }

        if (period > 1E-6) {
            double rawDerivative = (errorVal_p - prevErrorVal) / period;
            errorVal_v = (derivativeFilterGain * lastDerivative) + ((1 - derivativeFilterGain) * rawDerivative);
            lastDerivative = errorVal_v;
        } else {
            errorVal_v = 0;
        }

        if (errorVal_p * prevErrorVal < 0) {
            totalError = 0;
        }

        if (Math.abs(errorVal_p) <= integralZone) {
            totalError += period * errorVal_p;
            if (totalError > maxIntegral) totalError = maxIntegral;
            else if (totalError < minIntegral) totalError = minIntegral;
        } else {
            totalError = 0;
        }

        double mappedError = gainS.apply(errorVal_p);
        double mappedP = coefGain.apply(kP);

        double pTerm = mappedP * Math.sqrt(Math.abs(mappedError)) * Math.signum(mappedError);

        double fTerm = kF * setPoint;

        return pTerm + (kI * totalError) + (kD * errorVal_v) + fTerm;
    }

    public void setPIDF(double kp, double ki, double kd, double kf) {
        kP = kp;
        kI = ki;
        kD = kd;
        kF = kf;
    }

    public void setIntegrationBounds(double integralMin, double integralMax) {
        minIntegral = integralMin;
        maxIntegral = integralMax;
    }

    public void clearTotalError() {
        totalError = 0;
    }

    public void setP(double kp) { kP = kp; }
    public void setI(double ki) { kI = ki; }
    public void setD(double kd) { kD = kd; }
    public void setF(double kf) { kF = kf; }

    public double getP() { return kP; }
    public double getI() { return kI; }
    public double getD() { return kD; }
    public double getF() { return kF; }
    public double getPeriod() { return period; }
    public double getAngleMultiplier(){return angleMultiplier;}
}