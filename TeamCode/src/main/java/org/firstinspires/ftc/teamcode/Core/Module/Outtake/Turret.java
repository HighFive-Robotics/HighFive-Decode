package org.firstinspires.ftc.teamcode.Core.Module.Outtake;

import static org.firstinspires.ftc.teamcode.Constants.DeviceNames.turretMotorName;
import static org.firstinspires.ftc.teamcode.Constants.Globals.BlueGoalCorner;
import static org.firstinspires.ftc.teamcode.Constants.Globals.BlueGoalWallLeft;
import static org.firstinspires.ftc.teamcode.Constants.Globals.BlueGoalWallUp;
import static org.firstinspires.ftc.teamcode.Constants.Globals.RedGoalCorner;
import static org.firstinspires.ftc.teamcode.Constants.Globals.RedGoalWallRight;
import static org.firstinspires.ftc.teamcode.Constants.Globals.RedGoalWallUp;
import static org.firstinspires.ftc.teamcode.Constants.OuttakeConstants.TurretParams.kdTurret;
import static org.firstinspires.ftc.teamcode.Constants.OuttakeConstants.TurretParams.kfTurret;
import static org.firstinspires.ftc.teamcode.Constants.OuttakeConstants.TurretParams.kiTurret;
import static org.firstinspires.ftc.teamcode.Constants.OuttakeConstants.TurretParams.kpTurret;
import static org.firstinspires.ftc.teamcode.Constants.OuttakeConstants.TurretParams.maximumTicks;
import static org.firstinspires.ftc.teamcode.Constants.OuttakeConstants.TurretParams.minimumErrorAngleForWalls;
import static org.firstinspires.ftc.teamcode.Constants.OuttakeConstants.TurretParams.minimumTicks;
import static org.firstinspires.ftc.teamcode.Constants.OuttakeConstants.TurretParams.ticksPerPI;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Core.Hardware.HighModule;
import org.firstinspires.ftc.teamcode.Core.Hardware.HighMotor;

public class Turret extends HighModule {

    Telemetry telemetry;
    public HighMotor motor;
    public Constants.Color allianceColor;

    public enum States{
        Automated,
        Manual
    }

    States state = States.Automated;

    public double currentAngle, targetAngle, lastTargetAngle = 0, angleOffset = 0;
    public double ticks, targetTicks;

    public Turret(HardwareMap hw, Constants.Color allianceColor, Telemetry telemetry){
        motor = HighMotor.Builder.startBuilding()
                .setMotor(hw.get(DcMotorEx.class, turretMotorName))
                .setRunMode(HighMotor.RunMode.PID)
                .setReverseMotor(true)
                .setUseZeroPowerBehaviour(false)
                .setPIDCoefficients(kpTurret,kiTurret,kdTurret,kfTurret, HighMotor.FeedForwardType.Lift,1)
                .setEncoder(true,false)
                .build();
        motor.setTolerance(5);
        this.telemetry = telemetry;
        this.allianceColor = allianceColor;
    }

    /** This is the method that we will use to calculate the target angle of our turret.
     *  We use a simple formula based on the robot localization in the field, using trigonometry.
     *  We calculate the angle to 3 points of the goal, the corner and each wall and we see what point the shooter
     *  can be the most perpendicular. We do this by seeing what of this 3 points have the least error to their 3 ideal angles,
     *  this angles being 0, π/4 or π/2.
     * @param robotPose This is the pose of the robot, from which we use the x,y and heading.
     * @return this returns the target angle in Radians.
     */
    public double getTargetAngleFromDistance(Pose robotPose){
        double angleCorner, angleWallLeft, angleWallRight, angleWallUp;
        double errorCorner, errorWallLeft, errorWallRight, errorWallUp;
        if(allianceColor == Constants.Color.Blue){
            angleCorner = Math.atan2(BlueGoalCorner.getY() - robotPose.getY(), BlueGoalCorner.getX() - robotPose.getX());
            angleWallUp = Math.atan2(BlueGoalWallUp.getY() - robotPose.getY(), BlueGoalWallUp.getX() - robotPose.getX());
            angleWallLeft = Math.atan2(BlueGoalWallLeft.getY() - robotPose.getY(), BlueGoalWallLeft.getX() - robotPose.getX());

            errorCorner = Math.abs(-3*Math.PI/4-angleCorner);
            errorWallUp = Math.abs(-Math.PI/2-angleWallUp);
            errorWallLeft = Math.abs(-Math.PI - angleWallLeft);

            if(angleWallLeft <= minimumErrorAngleForWalls && angleWallLeft <= errorCorner){
                return angleWallLeft - robotPose.getHeading();
            } else if(errorWallUp <= minimumErrorAngleForWalls && angleWallUp <= errorCorner){
                return angleWallUp - robotPose.getHeading();
            } else {
                return angleCorner - robotPose.getHeading();
            }
        } else {
            angleCorner = Math.atan2(RedGoalCorner.getY() - robotPose.getY(), RedGoalCorner.getX() - robotPose.getX());
            angleWallUp = Math.atan2(RedGoalWallUp.getY() - robotPose.getY(), RedGoalWallUp.getX() - robotPose.getX());
            angleWallRight = Math.atan2(RedGoalWallRight.getY() - robotPose.getY(), RedGoalWallRight.getX() - robotPose.getX());

            errorCorner = Math.abs(Math.PI/4-angleCorner);
            errorWallUp = Math.abs(Math.PI/2-angleWallUp);
            errorWallRight = Math.abs(angleWallRight);//Because the formula is 0 - angleWallRight

            if(errorWallRight <= minimumErrorAngleForWalls && errorWallRight <= errorCorner){
                return angleWallRight - robotPose.getHeading();
            } else if(errorWallUp <= minimumErrorAngleForWalls && angleWallUp <= errorCorner){
                return angleWallUp - robotPose.getHeading();
            } else {
                return angleCorner - robotPose.getHeading();
            }
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
     *  This formula transform the angle which the turret rotates of [-7π/6,7π/6] in ticks from [-1.313,1.313].
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
        if (targetAngle <= - 5 * Math.PI / 6){
            if(Math.abs(targetAngle - currentAngle) >= Math.PI) {
                ticks = (1 + targetAngle / Math.PI) * ticksPerPI + ticksPerPI;
            }
        }
        ticks = Range.clip(ticks, minimumTicks, maximumTicks); // This is a fail safe so turret doesn't break
        return ticks;
    }

    /** This method sets the Target Angle to our turret, in which we calculate the Target Ticks, using the method calculateTargetTicksFromAngle().
     *  After we calculate the Target Ticks, we make a verification that if the difference isn't bigger than 1 degree or 0.017 radians to not change the PID target
     *  so we don't reset the integral sum that often, better loop time and for the [-7π/6,-π] and [π,7π/6] to not have bugs, when rotating to 5π/6 or to -5π/6.
     * @param targetInRadians This is the verification we make so the Target Angle that we want to set to our turret stays within the range of [-π,π].
     */
    @Override
    public void setTarget(double targetInRadians){
        targetInRadians = targetInRadians + angleOffset;
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
     *  so we don't reset the integral sum that often, better loop time and for the [-7π/6,-π] and [π,7π/6] to not have bugs, when rotating to 5π/6 or to -5π/6.
     * @param target This is the verification we make so the Target Angle that we want to set to our turret stays within the range of [-π,π].
     */
    public void setTargetDegrees(double target){
        target = Math.toRadians(target) + angleOffset;
        this.targetAngle = wrapAroundAngle(target);
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
     * @return This returns the current angle, which is in the range of [-7π/6,7π/6].
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
     * @return This returns the current angle, which is in the range of [-210°,210°].
     */
    public double getCurrentAngleDegrees(){
        return Math.toDegrees(currentAngle);
    }

    /** This method returns the angle of the turret, relative to the robot heading, which is in Degrees.
     * @return This returns the current angle, which is in the range of [-180°,180°].
     */
    public double getCurrentAngleWrappedDegrees(){
        return Math.toDegrees(wrapAroundAngle(currentAngle));
    }

    /** This method returns the current ticks of the turret, relative to the robot intake.
     * @return This returns the current ticks, which are in the range of [-1313,1313].
     */
    public double getCurrentTicks(){
        return ticks;
    }

    /** This method returns the target ticks of the turret, relative to the robot intake.
     * @return This returns the target ticks, which are in the range of [-1313,1313].
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
