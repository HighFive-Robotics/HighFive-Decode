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

    public HighMotor motor;
    public Constants.Color allianceColor;

    public enum States{
        Automated,
        Manual
    }

    States state = States.Automated;

    public double currentAngle, targetAngle, lastTargetAngle = 0, angleOffset = 0;
    public double ticks, targetTicks;

    public Turret(HardwareMap hw, Constants.Color allianceColor){
        motor = HighMotor.Builder.startBuilding()
                .setMotor(hw.get(DcMotorEx.class, turretMotorName))
                .setRunMode(HighMotor.RunMode.PID)
                .setReverseMotor(true)
                .setUseZeroPowerBehaviour(false)
                .setPIDCoefficients(kpTurret,kiTurret,kdTurret,kfTurret, HighMotor.FeedForwardType.Lift,1)
                .setEncoder(true,false)
                .build();
        motor.setTolerance(5);
        this.allianceColor = allianceColor;
    }

    /** This is the method that we will use to calculate the target angle of our turret.
     *  We use a simple formula based on the robot localization in the field, using trigonometry.
     * @param robotPose This is the pose of the robot, from which we use the x,y and heading.
     * @return this returns the target angle in Radians.
     */
    public double getTargetAngleFromDistance(Pose robotPose){
        if(allianceColor == Constants.Color.Blue){
            return Math.atan2(BlueGoal.getY() - robotPose.getY(), BlueGoal.getX() - robotPose.getX()) - robotPose.getHeading();
        } else {
            return Math.atan2(RedGoal.getY() - robotPose.getY(), RedGoal.getX() - robotPose.getX()) - robotPose.getHeading();
        }
    }

    /** This method makes sure that our angle is in the range of [-π,π].
     * @param angle This is the angle that we will wrap around.
     * @return this is the value of the angle in the [-π,π] range.
     */
    public double wrapAroundAngle(double angle){
        while(angle < -Math.PI){
            angle += 2 * Math.PI;
        }
        while(angle > Math.PI){
            angle -= 2 * Math.PI;
        }
        return angle;
    }

    /** This method calculates the ticks target from the target angle that we want, making sure that our turret does not rotate past 11π/9 in both directions.
     *  This formula transform the angle which the turret rotates of [-11π/9,11π/9] in ticks from [-1375,1375].
     *  The Target Angle that we want to set to our turret is within the range of [-π,π], verification made before the method is used.
     */
    public double calculateTargetTicksFromAngle(){
        double ticks;
        ticks = targetAngle / Math.PI * ticksPerPI;
        if (targetAngle >= 7 * Math.PI / 9){
            if(Math.abs(targetAngle - currentAngle) >= Math.PI){
                ticks = (targetAngle / Math.PI - 1) * ticksPerPI - ticksPerPI;
            }
        }
        if (targetAngle <= - 7 * Math.PI / 9){
            if(Math.abs(targetAngle - currentAngle) >= Math.PI) {
                ticks = (1 + targetAngle / Math.PI) * ticksPerPI + ticksPerPI;
            }
        }
        ticks = Range.clip(ticks, minimumTicks, maximumTicks); // This is a fail safe so turret doesn't break
        return ticks;
    }

    /** This method sets the Target Angle to our turret, in which we calculate the Target Ticks, using the method calculateTargetTicksFromAngle().
     *  After we calculate the Target Ticks, we make a verification that if the difference isn't bigger than 1 degree or 0.017 radians to not change the PID target
     *  so we don't reset the integral sum that often, better loop time and for the [-11π/9,-π] and [π,11π/9] to not have bugs, when rotating to 7π/9 or to -7π/9.
     * @param targetInRadians This is the verification we make so the Target Angle that we want to set to our turret stays within the range of [-π,π].
     */
    @Override
    public void setTarget(double targetInRadians){
        targetAngle += angleOffset;
        this.targetAngle = wrapAroundAngle(targetInRadians);
        targetTicks = calculateTargetTicksFromAngle();
        if(Math.abs(targetAngle - lastTargetAngle) >= 0.017){
            motor.setTarget(targetTicks);
            lastTargetAngle = targetAngle;
        } else {
            targetAngle = lastTargetAngle;
        }
    }

    /** This method sets the Target ticks to our turret.
     *  We use this method for calibration only.
     * @param target This is the target in ticks.
     */
    public void setTargetTicks(double target){
        targetTicks = Range.clip(target, minimumTicks, maximumTicks); // This is a fail safe so turret doesn't break
        motor.setTarget(targetTicks);
        lastTargetAngle = targetAngle;
    }

    /** This method sets the Target Angle in degrees to our turret, in which we calculate the Target Ticks, using the method calculateTargetTicksFromAngle().
     *  After we calculate the Target Ticks, we make a verification that if the difference isn't bigger than 1 degree or 0.017 radians to not change the PID target
     *  so we don't reset the integral sum that often, better loop time and for the [-11π/9,-π] and [π,11π/9] to not have bugs, when rotating to 7π/9 or to -7π/9.
     * @param target This is the verification we make so the Target Angle that we want to set to our turret stays within the range of [-π,π].
     */
    public void setTargetDegrees(double target){
        this.targetAngle = Math.toRadians(target);
        targetAngle += angleOffset;
        this.targetAngle = wrapAroundAngle(targetAngle);
        targetTicks = calculateTargetTicksFromAngle();
        if(Math.abs(targetAngle - lastTargetAngle) >= 0.017){
            motor.setTarget(targetTicks);
            lastTargetAngle = targetAngle;
        } else {
            targetAngle = lastTargetAngle;
        }
    }

    /** In this method we set the offset of the turret in Radians.
     * @param offsetInRadians this is the offset of the turret in Radians that we add to the target when we set one.
     */
    public void setOffset(double offsetInRadians){
        angleOffset = offsetInRadians;
    }

    /** In this method we set the offset of the turret in Degrees.
     * @param offset this is the offset of the turret in Degrees that we add to the target when we set one.
     */
    public void setOffsetDegrees(double offset){
        angleOffset = Math.toRadians(offset);
    }

    /** In this method we add the offset of the turret in Radians.
     * @param offsetInRadians this is the offset what will be added to the turret in Radians, that we add to the target when we set one.
     */
    public void addOffset(double offsetInRadians){
        angleOffset += offsetInRadians;
    }

    /** In this method we add the offset of the turret in Degrees.
     * @param offset this is the offset what will be added to the turret in Degrees, that we add to the target when we set one.
     */
    public void addOffsetDegrees(double offset){
        angleOffset += Math.toRadians(offset);
    }

    /** From this method we get the angle offset of the turret.
     * @return This returns the angle offset
     */
    public double getOffset(){
        return angleOffset;
    }

    /** From this method you will know if the turret has reached the target.
     * @return This returns if the total error of the turret is less than 1.5 Degrees or 0.025 Radians.
     */
    @Override
    public boolean atTarget(){
        return Math.abs(targetAngle - currentAngle) >= 0.025;
    }

    /** This method returns the target of the turret, which is in Radians.
     * @return This returns the target, which is in the range of [-π,π].
     */
    @Override
    public double getTarget(){
        return targetAngle;
    }

    /** This method returns the target of the turret, which is in Radians.
     * @return This returns the target, which is in the range of [-180°,180°].
     */
    public double getTargetDegrees(){
        return Math.toDegrees(targetAngle);
    }

    /** This method returns the angle of the turret, relative to the robots intake, which is in Radians.
     * @return This returns the current angle, which is in the range of [-11π/9,11π/9].
     */
    public double getCurrentAngle(){
        return currentAngle;
    }

    /** This method returns the angle of the turret, relative to the robot heading, which is in Radians.
     * @return This returns the current angle, which is in the range of [-π,π].
     */
    public double getCurrentWrappedAngle(){
        return wrapAroundAngle(currentAngle);
    }

    /** This method returns the angle of the turret, relative to the robot intake, which is in Degrees.
     * @return This returns the current angle, which is in the range of [-220°,220°].
     */
    public double getCurrentAngleDegrees(){
        return Math.toDegrees(currentAngle);
    }

    /** This method returns the angle of the turret, relative to the robot heading, which is in Degrees.
     * @return This returns the current angle, which is in the range of [-180°,180°].
     */
    public double getCurrentAngleWrappedDegrees(){
        return Math.toDegrees(currentAngle);
    }

    /** This method returns the current ticks of the turret, relative to the robot intake.
     * @return This returns the current ticks, which are in the range of [-1375,1375].
     */
    public double getCurrentTicks(){
        return ticks;
    }

    /** This method returns the target ticks of the turret, relative to the robot intake.
     * @return This returns the target ticks, which are in the range of [-1375,1375].
     */
    public double getTargetTicks(){
        return targetTicks;
    }

    /** This method resets the motor encoder and sets the target to 0.*/
    public void reset(){
        lastTargetAngle = 0;
        angleOffset = 0;
        motor.resetMotor();
        motor.setTarget(0);
    }

    @Override
    public void update() {
        ticks = motor.getCurrentPosition();
        currentAngle = (ticks / ticksPerPI) * Math.PI;
        motor.update();
    }
}
