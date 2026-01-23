package org.firstinspires.ftc.teamcode.OpModes.Autos;

import static org.firstinspires.ftc.teamcode.Constants.Globals.autoColor;
import static org.firstinspires.ftc.teamcode.Constants.Globals.finalAutoPose;
import static org.firstinspires.ftc.teamcode.Core.Module.Intake.IntakeMotor.States.Collect;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Core.Module.Intake.Intake;
import org.firstinspires.ftc.teamcode.Core.Module.Outtake.BlockerOuttake;
import org.firstinspires.ftc.teamcode.Core.Robot;

@Autonomous(name = "🔴AutoFar🔴")
public class AutoRedFar extends LinearOpMode {

    public Robot robot;
    public int state = 0;
    public double velocityFar = 4.8, velocityNeg = - 1.5;

    public Pose startPose = new Pose(86.5, 10, Math.toRadians(-90));
    public Pose shootPose = new Pose( 82.5,17, Math.toRadians(-110));
    public Pose collectPose = new Pose(130, 22, Math.toRadians(85));
    public Pose parkPose = new Pose(100, 17, Math.toRadians(150));
    public Pose controlPoint1 = new Pose(115, 4);

    private final ElapsedTime autoTimer = new ElapsedTime();
    private final ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, startPose, true, Constants.Color.Red, telemetry, gamepad1);
        autoColor = Constants.Color.Red;
        robot.drive.resetTeleOpHeading();
        robot.camera.startCapture();
        robot.drive.setConstants(Constants.FConstants);
        robot.intake.setCollectType(Intake.CollectTypes.Normal);
        robot.intake.setState(Intake.States.Wait);

        PathChain preloadPath = robot.drive.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        PathChain goCollect = robot.drive.pathBuilder()
                .addPath(new BezierCurve(
                        shootPose,
                        controlPoint1,
                        collectPose
                )).setTangentHeadingInterpolation()
                .build();

        PathChain goPark = robot.drive.pathBuilder()
                .addPath(new BezierLine(collectPose, parkPose))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        PathChain goShoot = robot.drive.pathBuilder()
                .addPath(new BezierLine(parkPose, shootPose))
                .setLinearHeadingInterpolation(parkPose.getHeading(), shootPose.getHeading())
                .build();

        Constants.Globals.afterAuto = true;
        telemetry.addLine("Ready for Action");
        telemetry.update();
        waitForStart();
        autoTimer.reset();
        while (opModeIsActive()) {
            switch (state) {
                case 0:
                    robot.drive.setMaxPower(0.7);
                    robot.drive.followPath(preloadPath, true);
                    robot.shooter.setTargetVelocity(velocityFar);
                    robot.shooter.blocker.setState(BlockerOuttake.States.Open);
                    state++;
                    break;
                case 1:
                    if((robot.isDone() && robot.shooter.atTarget()) || autoTimer.milliseconds() > 5000) {
                        robot.setAction(Robot.Actions.ShootFastNormal);
                        timer.reset();
                        state++;
                    }
                    break;
                case 2:
                    if(!robot.shootNormal || timer.milliseconds() >= 15000){
                        robot.drive.setMaxPower(1);
                        robot.intake.setState(Intake.States.Wait);
                        robot.intake.setCollectType(Intake.CollectTypes.Normal);
                        robot.shooter.blocker.setState(BlockerOuttake.States.Open);
                        robot.intake.setPower(Collect);
                        robot.shooter.setTargetVelocity(velocityNeg);
                        robot.drive.followPath(goCollect, true);
                        timer.reset();
                        state++;
                    }
                    break;
                case 3:
                    if(timer.milliseconds() >= 6000){
                        robot.drive.followPath(goPark, true);
                        state++;
                    }
                    break;
                case 4:
                    if(robot.drive.atParametricEnd()){
                        robot.drive.followPath(goShoot, true);
                        state++;
                    }
                    break;
                case 5:
                    if(robot.isDone()){
                        robot.setAction(Robot.Actions.PrepareForShooting);
                        state++;
                        timer.reset();
                    }
                    break;
                case 6:
                    if(timer.milliseconds() >= 200){
                        robot.shooter.setTargetVelocity(velocityFar);
                        state++;
                        timer.reset();
                    }
                    break;
                case 7:
                    if((robot.isDone() && robot.shooter.atTarget()) || timer.milliseconds() > 5000) {
                        robot.setAction(Robot.Actions.ShootFastNormal);
                        timer.reset();
                        state++;
                    }
                    break;
                case 8:
                    if(!robot.shootNormal || timer.milliseconds() >= 15000){
                        robot.intake.setPower(Collect);
                        robot.shooter.setTargetVelocity(velocityNeg);
                        robot.drive.followPath(goCollect, true);
                        timer.reset();
                        state++;
                    }
                    break;
                case 9:
                    if(timer.milliseconds() >= 6500){
                        robot.drive.followPath(goPark, true);
                        state++;
                    }
                    break;
                case 10:
                    if(robot.isDone()){
                        robot.setAction(Robot.Actions.PrepareForShooting);
                        state++;
                    }
                    break;
            }

            finalAutoPose = robot.drive.getPose();
            robot.update();
            telemetry.addData("State: ", state);
            telemetry.update();
        }
    }
}
