package org.firstinspires.ftc.teamcode.Core.Module.Outtake;

import static org.firstinspires.ftc.teamcode.Constants.DeviceNames.turretMotorName;
import static org.firstinspires.ftc.teamcode.Constants.Globals.BlueGoal;
import static org.firstinspires.ftc.teamcode.Constants.Globals.RedGoal;
import static org.firstinspires.ftc.teamcode.Constants.OuttakeConstants.TurretParams.kdTurret;
import static org.firstinspires.ftc.teamcode.Constants.OuttakeConstants.TurretParams.kfTurret;
import static org.firstinspires.ftc.teamcode.Constants.OuttakeConstants.TurretParams.kiTurret;
import static org.firstinspires.ftc.teamcode.Constants.OuttakeConstants.TurretParams.kpTurret;
import static org.firstinspires.ftc.teamcode.Constants.OuttakeConstants.TurretParams.maximumTicks;
import static org.firstinspires.ftc.teamcode.Constants.OuttakeConstants.TurretParams.minimumTicks;
import static org.firstinspires.ftc.teamcode.Constants.OuttakeConstants.TurretParams.ticksPerPI;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Core.Hardware.HighModule;
import org.firstinspires.ftc.teamcode.Core.Hardware.HighMotor;

public class Turret extends HighModule {

    HighMotor motor;
    Constants.Color allianceColor;

    public enum States{
        Automated,
        Manual
    }

    public double currentAngle, targetAngle, angleOffset;
    public double ticks, targetTicks, lastTargetTicks = 0;

    public Turret(HardwareMap hw, Constants.Color allianceColor){
        motor = HighMotor.Builder.startBuilding()
                .setMotor(hw.get(DcMotorEx.class, turretMotorName))
                .setRunMode(HighMotor.RunMode.PID)
                .setReverseMotor(true)
                .setUseZeroPowerBehaviour(false)
                .setPIDCoefficients(kpTurret,kiTurret,kdTurret,kfTurret, HighMotor.FeedForwardType.Lift,1)
                .setEncoder(true,false)
                .build();
        motor.setTolerance(10);
        this.allianceColor = allianceColor;
    }

    public double getTargetAngleFromDistance(Pose robotPose){
        if(allianceColor == Constants.Color.Blue){
            return Math.atan2(BlueGoal.getY() - robotPose.getY(), BlueGoal.getX() - robotPose.getX()) - robotPose.getHeading();
        } else {
            return Math.atan2(RedGoal.getY() - robotPose.getY(), RedGoal.getX() - robotPose.getX()) - robotPose.getHeading();
        }
    }

    public double wrapAroundAngle(double angle){
        while(angle < -Math.PI){
            angle += 2 * Math.PI;
        }
        while(angle > Math.PI){
            angle -= 2 * Math.PI;
        }
        return angle;
    }

    /** This method calculates the ticks target from the target angle that we want, making sure that our turret does not rotate past 7π/6 in both directions.
     *  This formulas transform the angle which the turret rotates of [-7π/6, 7π/6] in ticks from [-1400, 1400].
     *  The Target Angle that we want to set to our turret is within the range of [-π,π], verification made before the method is used.
     */
    public double calculateTargetTicksFromAngle(){
        double ticks;
        ticks = targetAngle / Math.PI * ticksPerPI;
        if (targetAngle >= 5 * Math.PI / 6){
            if(Math.abs(targetAngle - currentAngle) >= Math.PI){
                ticks = (targetAngle / Math.PI - 1) * ticksPerPI - ticksPerPI;
            }
        }
        if (targetAngle < - 5 * Math.PI / 6){
            if(Math.abs(targetAngle - currentAngle) >= Math.PI) {
                ticks = (1 + targetAngle / Math.PI) * ticksPerPI + ticksPerPI;
            }
        }
        ticks = Range.clip(targetTicks, minimumTicks, maximumTicks); // This is a fail safe so turret doesn't break
        return ticks;
    }

    /** This method sets the Target Angle to our turret, in which we calculate the Target Ticks, using the method calculateTargetTicksFromAngle().
     *  After we calculate the Target Ticks, we make a verification that if the difference isn't bigger than 2 to not change the PID target
     *  so we don't reset the integral sum that often and better loop time.
     * @param target This is the verification we make so the Target Angle that we want to set to our turret stays within the range of [-π,π].
     */
    public void setTargetAngle(double target){
        targetAngle += angleOffset;
        this.targetAngle = wrapAroundAngle(target);
        targetTicks = calculateTargetTicksFromAngle();
        if(Math.abs(targetTicks - lastTargetTicks) >= 2){
            motor.setTarget(targetTicks);
            lastTargetTicks = targetTicks;
        } else {
            targetTicks = lastTargetTicks;
        }
    }

    /** This method sets the Target Angle in degrees to our turret, in which we calculate the Target Ticks, using the method calculateTargetTicksFromAngle().
     *  After we calculate the Target Ticks, we make a verification that if the difference isn't bigger than 2 to not change the PID target
     *  so we don't reset the integral sum that often and better loop time.
     * @param target This is the verification we make so the Target Angle that we want to set to our turret stays within the range of [-π,π].
     */
    public void setTargetAngleDegrees(double target){
        this.targetAngle = Math.toRadians(target);
        targetAngle += angleOffset;
        this.targetAngle = wrapAroundAngle(targetAngle);
        targetTicks = calculateTargetTicksFromAngle();
        if(Math.abs(targetTicks - lastTargetTicks) >= 2){
            motor.setTarget(targetTicks);
            lastTargetTicks = targetTicks;
        } else {
            targetTicks = lastTargetTicks;
        }
    }

    @Override
    public void update() {
        ticks = motor.getCurrentPosition();
        currentAngle = (ticks / ticksPerPI) * Math.PI;
        currentAngle = wrapAroundAngle(currentAngle);
        motor.update();
    }
}
