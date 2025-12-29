package org.firstinspires.ftc.teamcode.Core.Algorithms;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.util.Range;

public class HighController {

    public enum Modes {
        Axial,
        Angular,
    }

    public static class Kernel {
        public double kP = 0, kI = 0, kD = 0;
        public double staticGain = 0;
        public double dynamicGain = 0;
        public double inertialGain = 0;
        public double cosineGain = 0;
        public double filterGain = 0.8;
        public double tolerance = 1.0;
        public double bangBangRange = 0;
        public double bangBangGain = 1;
        public double precisionThreshold = 15.0;
        public double integralActivationZone = 30.0;
        public double outputSlewRate = 0.0;
        public double stuckDerivativeThreshold = 0.5;
        public double stuckTimeout = 0.25;
        public double maxIntegralSum = 1.0;
        public double lookaheadTime = 0.1;
        public double brakingGain = 2.0;
        public double spinOpposingGain = 1.11;
        public double spinOpposingPrefix = -1.0;
        public boolean useOpposingCorrection = true;
        public double precisionMin = 0.5;
        public double precisionMax = 1;
        public double viscousGain = 1;

        public void setPID(double kP, double kI, double kD) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
        }

        public void setViscousGain(double viscousGain) {
            this.viscousGain = viscousGain;
        }

        public void setPrecisionAddition(double min, double max) {
            this.precisionMin = min;
            this.precisionMax = max;
        }

        public void setOpposingCorrection(boolean useOpposingCorrection, double spinOpposingGain, double spinOpposingPrefix) {
            this.useOpposingCorrection = useOpposingCorrection;
            this.spinOpposingGain = spinOpposingGain;
            this.spinOpposingPrefix = spinOpposingPrefix;
        }

        public void setFeedForward(double staticGain, double dynamicGain, double inertialGain) {
            this.inertialGain = inertialGain;
            this.staticGain = staticGain;
            this.dynamicGain = dynamicGain;
        }

        public void setFeedForward(double staticGain, double dynamicGain, double inertialGain, double cosineGain) {
            this.staticGain = staticGain;
            this.dynamicGain = dynamicGain;
            this.inertialGain = inertialGain;
            this.cosineGain = cosineGain;
        }

        public void setBangBangRange(double range) {
            this.bangBangRange = range;
        }

        public void setPrecisionThreshold(double threshold) {
            this.precisionThreshold = threshold;
        }

        public void setStuckProtection(double threshold, double timeout) {
            this.stuckDerivativeThreshold = threshold;
            this.stuckTimeout = timeout;
        }

        public static void copy(Kernel k1, @NonNull Kernel k2) {
            k1.kP = k2.kP;
            k1.kD = k2.kD;
            k1.kI = k2.kI;
            k1.inertialGain = k2.inertialGain;
            k1.staticGain = k2.staticGain;
            k1.dynamicGain = k2.dynamicGain;
            k1.cosineGain = k2.cosineGain;
            k1.filterGain = k2.filterGain;
            k1.tolerance = k2.tolerance;
            k1.bangBangRange = k2.bangBangRange;
            k1.stuckDerivativeThreshold = k2.stuckDerivativeThreshold;
            k1.stuckTimeout = k2.stuckTimeout;
            k1.maxIntegralSum = k2.maxIntegralSum;
            k1.integralActivationZone = k2.integralActivationZone;
            k1.lookaheadTime = k2.lookaheadTime;
            k1.brakingGain = k2.brakingGain;
            k1.precisionThreshold = k2.precisionThreshold;
            k1.outputSlewRate = k2.outputSlewRate;
        }

        public static class Builder {
            private Kernel kernel = new Kernel();

            public Builder setPID(double kP, double kI, double kD) {
                kernel.kP = kP;
                kernel.kI = kI;
                kernel.kD = kD;
                return this;
            }

            public Builder setPrecisionAddition(double min, double max) {
                kernel.precisionMin = min;
                kernel.precisionMax = max;
                return this;
            }
            public Builder setViscousGain(double viscousGain) {
                kernel.viscousGain = viscousGain;
                return this;
            }

            public Builder setFeedforward(double staticGain, double dynamicGain, double inertialGain) {
                kernel.staticGain = staticGain;
                kernel.dynamicGain = dynamicGain;
                kernel.inertialGain = inertialGain;
                return this;
            }

            public Builder setFeedforward(double staticGain, double dynamicGain, double inertialGain, double cosineGain) {
                kernel.staticGain = staticGain;
                kernel.dynamicGain = dynamicGain;
                kernel.inertialGain = inertialGain;
                kernel.cosineGain = cosineGain;
                return this;
            }

            public Builder setFilterGain(double filterGain) {
                kernel.filterGain = filterGain;
                return this;
            }

            public Builder setTolerance(double tolerance) {
                kernel.tolerance = tolerance;
                return this;
            }

            public Builder setBangBangRange(double range) {
                kernel.bangBangRange = range;
                return this;
            }

            public Builder setBangBangGain(double gain) {
                kernel.bangBangGain = gain;
                return this;
            }

            public Builder setStuckProtection(double threshold, double timeout) {
                kernel.stuckDerivativeThreshold = threshold;
                kernel.stuckTimeout = timeout;
                return this;
            }

            public Builder setAntiWindupMax(double max) {
                kernel.maxIntegralSum = max;
                return this;
            }

            public Builder setIntegralZone(double zone) {
                kernel.integralActivationZone = zone;
                return this;
            }

            public Builder setPrecisionThreshold(double threshold) {
                kernel.precisionThreshold = threshold;
                return this;
            }

            public Builder setOscillationDampener(double lookahead, double gain) {
                kernel.lookaheadTime = lookahead;
                kernel.brakingGain = gain;
                return this;
            }

            public Builder setSlewRate(double maxChangePerSecond) {
                kernel.outputSlewRate = maxChangePerSecond;
                return this;
            }

            public Builder setOpposingCorrection(boolean useOpposingCorrection, double spinOpposingGain, double spinOpposingPrefix) {
                kernel.useOpposingCorrection = useOpposingCorrection;
                kernel.spinOpposingGain = spinOpposingGain;
                kernel.spinOpposingPrefix = spinOpposingPrefix;
                return this;
            }

            public Kernel build() {
                return kernel;
            }
        }
    }

    public Kernel params = new Kernel();
    private double target = 0;
    private Modes mode = Modes.Axial;

    private double lastError = 0;
    private double integralSum = 0;
    private double lastPosition = 0;
    private double lastVelocity = 0;
    private double lastOutput = 0;
    private long lastTimestamp = 0;

    private double stuckTimerStart = 0;
    private boolean isStuck = false;

    public HighController(Kernel params) {
        this.params = params;
    }

    public HighController(Kernel params, Modes mode) {
        this.params = params;
        this.setMode(mode);
    }

    public void setKernel(Kernel params) {
        this.params = params;
    }

    public void setMode(Modes mode) {
        this.mode = mode;
    }

    public void setTarget(double target) {
        this.target = target;
        this.isStuck = false;
        this.stuckTimerStart = 0;
    }

    public double getError() {
        return lastError;
    }

    public void reset() {
        integralSum = 0;
        lastError = 0;
        lastPosition = 0;
        lastVelocity = 0;
        lastOutput = 0;
        lastTimestamp = 0;
        isStuck = false;
        stuckTimerStart = 0;
    }

    public double run(double currentPosition, double currentVelocity) {
        long currentTimestamp = System.nanoTime();

        if (lastTimestamp == 0) {
            lastTimestamp = currentTimestamp;
            lastError = delta(currentPosition, target);
            lastPosition = currentPosition;
            return 0.0;
        }

        double dt = (currentTimestamp - lastTimestamp) * 1.0E-9;
        if (dt < 1.0E-6) dt = 1.0E-6;

        double error = delta(currentPosition, target);
        double absError = Math.abs(error);
        double signError = Math.signum(error);

        if (params.bangBangRange > 0 && absError > params.bangBangRange) {
            integralSum = 0;
            lastError = error;
            lastTimestamp = currentTimestamp;
            lastPosition = currentPosition;
            double output = signError * params.bangBangGain;
            if (params.outputSlewRate > 0) {
                double maxChange = params.outputSlewRate * dt;
                output = Range.clip(output, lastOutput - maxChange, lastOutput + maxChange);
            }
            lastOutput = output;
            return output;
        }

        if (absError <= params.tolerance) {
            integralSum = 0;
            lastError = error;
            lastTimestamp = currentTimestamp;
            lastPosition = currentPosition;
            lastVelocity = 0;
            double gravity = params.inertialGain + (params.cosineGain * Math.cos(Math.toRadians(currentPosition)));
            lastOutput = gravity;
            return gravity;
        }

        double pTerm;
        if (absError < params.precisionThreshold) {
            double precisionAddition = Range.scale(params.precisionThreshold, 0, 360, params.precisionMin, params.precisionMax);
            double preciseError = absError * (1 + precisionAddition);
            pTerm = params.kP * Math.pow(preciseError, 1.2) * signError;
        } else {
            pTerm = params.kP * Math.pow(absError, 1.2) * signError;
        }

        if (absError < params.integralActivationZone) {
            integralSum += (error * dt);
        } else {
            integralSum = 0;
        }
        if (Math.signum(error) != Math.signum(lastError)) integralSum = 0;

        if (params.kI != 0) {
            double maxISum = params.maxIntegralSum / params.kI;
            integralSum = Range.clip(integralSum, -maxISum, maxISum);
        }
        double iTerm = params.kI * integralSum;

        double positionChange = currentPosition - lastPosition;
        if (mode == Modes.Angular) {
            positionChange %= 360;
            if (positionChange > 180) positionChange -= 360;
            else if (positionChange < -180) positionChange += 360;
        }

        double filteredVelocity = (params.filterGain * currentVelocity) + ((1.0 - params.filterGain) * lastVelocity);
        double dTerm = -params.kD * filteredVelocity;

        double feedforward = (params.staticGain * signError)
                + params.inertialGain
                + (params.cosineGain * Math.cos(Math.toRadians(currentPosition)))
                + (params.dynamicGain * -filteredVelocity);

        if (absError > params.tolerance && Math.abs(filteredVelocity) < params.stuckDerivativeThreshold) {
            if (stuckTimerStart == 0) stuckTimerStart = currentTimestamp * 1.0E-9;
            else if ((currentTimestamp * 1.0E-9) - stuckTimerStart > params.stuckTimeout)
                isStuck = true;
        } else {
            stuckTimerStart = 0;
            if (absError <= params.tolerance) isStuck = false;
        }
        if (Math.signum(params.spinOpposingPrefix) == Math.signum(error) && params.useOpposingCorrection) {
            pTerm *= params.spinOpposingGain;
            dTerm *= params.spinOpposingGain;
        }
        if (absError <= params.precisionThreshold) {
            dTerm *= params.viscousGain;
        }
        double output = pTerm + iTerm + dTerm + feedforward;

        double projectedError = error - (filteredVelocity * params.lookaheadTime);

        if (Math.signum(projectedError) != Math.signum(error)) {
            double overshootMag = Math.abs(projectedError);
            output /= (1.0 + (overshootMag * params.brakingGain));
        }

        if (params.outputSlewRate > 0) {
            double maxChange = params.outputSlewRate * dt;
            output = Range.clip(output, lastOutput - maxChange, lastOutput + maxChange);
        }

        output = Range.clip(output, -1.0, 1.0);

        lastTimestamp = currentTimestamp;
        lastError = error;
        lastPosition = currentPosition;
        lastVelocity = filteredVelocity;
        lastOutput = output;

        return output;
    }

    public double run(double currentPosition) {
        long currentTimestamp = System.nanoTime();

        if (lastTimestamp == 0) {
            lastTimestamp = currentTimestamp;
            lastError = delta(currentPosition, target);
            lastPosition = currentPosition;
            return 0.0;
        }

        double dt = (currentTimestamp - lastTimestamp) * 1.0E-9;
        if (dt < 1.0E-6) dt = 1.0E-6;

        double error = delta(currentPosition, target);
        double absError = Math.abs(error);
        double signError = Math.signum(error);

        if (params.bangBangRange > 0 && absError > params.bangBangRange) {
            integralSum = 0;
            lastError = error;
            lastTimestamp = currentTimestamp;
            lastPosition = currentPosition;
            double output = signError * params.bangBangGain;
            if (params.outputSlewRate > 0) {
                double maxChange = params.outputSlewRate * dt;
                output = Range.clip(output, lastOutput - maxChange, lastOutput + maxChange);
            }
            lastOutput = output;
            return output;
        }

        if (absError <= params.tolerance) {
            integralSum = 0;
            lastError = error;
            lastTimestamp = currentTimestamp;
            lastPosition = currentPosition;
            lastVelocity = 0;
            double gravity = params.inertialGain + (params.cosineGain * Math.cos(Math.toRadians(currentPosition)));
            lastOutput = gravity;
            return gravity;
        }

        double pTerm;
        if (absError < params.precisionThreshold) {
            double precisionAddition = Range.scale(params.precisionThreshold, 0, 360, params.precisionMin, params.precisionMax);
            double preciseError = absError * (1 + precisionAddition);
            pTerm = params.kP * Math.pow(preciseError, 1.2) * signError;
        } else {
            pTerm = params.kP * Math.pow(absError, 1.2) * signError;
        }

        if (absError < params.integralActivationZone) {
            integralSum += (error * dt);
        } else {
            integralSum = 0;
        }
        if (Math.signum(error) != Math.signum(lastError)) integralSum = 0;

        if (params.kI != 0) {
            double maxISum = params.maxIntegralSum / params.kI;
            integralSum = Range.clip(integralSum, -maxISum, maxISum);
        }
        double iTerm = params.kI * integralSum;

        double positionChange = currentPosition - lastPosition;
        if (mode == Modes.Angular) {
            positionChange %= 360;
            if (positionChange > 180) positionChange -= 360;
            else if (positionChange < -180) positionChange += 360;
        }

        double rawVelocity = positionChange / dt;
        double filteredVelocity = (params.filterGain * rawVelocity) + ((1.0 - params.filterGain) * lastVelocity);
        double dTerm = -params.kD * filteredVelocity;

        double feedforward = (params.staticGain * signError)
                + params.inertialGain
                + (params.cosineGain * Math.cos(Math.toRadians(currentPosition)))
                + (params.dynamicGain * -filteredVelocity);

        if (absError > params.tolerance && Math.abs(filteredVelocity) < params.stuckDerivativeThreshold) {
            if (stuckTimerStart == 0) stuckTimerStart = currentTimestamp * 1.0E-9;
            else if ((currentTimestamp * 1.0E-9) - stuckTimerStart > params.stuckTimeout)
                isStuck = true;
        } else {
            stuckTimerStart = 0;
            if (absError <= params.tolerance) isStuck = false;
        }
        if (Math.signum(params.spinOpposingPrefix) == Math.signum(error) && params.useOpposingCorrection) {
            pTerm *= params.spinOpposingGain;
            dTerm *= params.spinOpposingGain;
        }
        if (absError <= params.precisionThreshold) {
            dTerm *= params.viscousGain;
        }
        double output = pTerm + iTerm + dTerm + feedforward;

        double projectedError = error - (filteredVelocity * params.lookaheadTime);

        if (Math.signum(projectedError) != Math.signum(error)) {
            double overshootMag = Math.abs(projectedError);
            output /= (1.0 + (overshootMag * params.brakingGain));
        }

        if (params.outputSlewRate > 0) {
            double maxChange = params.outputSlewRate * dt;
            output = Range.clip(output, lastOutput - maxChange, lastOutput + maxChange);
        }

        output = Range.clip(output, -1.0, 1.0);

        lastTimestamp = currentTimestamp;
        lastError = error;
        lastPosition = currentPosition;
        lastVelocity = filteredVelocity;
        lastOutput = output;

        return output;
    }

    public boolean isBusy() {
        return Math.abs(lastError) > params.tolerance;
    }

    public boolean isStuck() {
        return isStuck;
    }

    public double delta(double position, double target) {
        if (mode == Modes.Axial) return target - position;
        double difference = (target - position) % 360;
        if (difference > 180) difference -= 360;
        if (difference < -180) difference += 360;
        return difference;
    }
}