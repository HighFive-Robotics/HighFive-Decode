package org.firstinspires.ftc.teamcode.OpModes.Autos;

import static org.firstinspires.ftc.teamcode.Constants.Globals.autoColor;
import static org.firstinspires.ftc.teamcode.Constants.Globals.finalAutoPose;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;

import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Core.Module.Intake.Intake;
import org.firstinspires.ftc.teamcode.Core.Module.Intake.IntakeMotor;
import org.firstinspires.ftc.teamcode.Core.Module.Outtake.BlockerOuttake;
import org.firstinspires.ftc.teamcode.Core.Robot;

@Autonomous(name = "🔵AutoClose🔵")
public class AutoBlue extends LinearOpMode {

    public Robot robot;
    public int state = 0;
    public double velocityLast = 3.3, velocityPreload = 3.6, velocityFar = 3.5, velocityNeg = - 1.5;

    public Pose startPose = new Pose(15, 111, Math.toRadians(0));
    public Pose shootPose = new Pose(55, 85, Math.toRadians(-42));
    public Pose lastShootPose = new Pose(50, 112.5, Math.toRadians(-25));
    public Pose preOpenGatePose = new Pose(20, 68, Math.toRadians(-90));
    public Pose openGatePose = new Pose(13.5, 68, Math.toRadians(-90));
    public Pose controlPoint1 = new Pose(60, 60);
    public Pose controlPoint2 = new Pose(45, 65);
    public Pose controlPoint3 = new Pose(60, 40);
    public Pose preCollectSpikeMark2Pose = new Pose(42.5, 60, Math.toRadians(180));
    public Pose collectSpikeMark2Pose = new Pose(17, 60, Math.toRadians(180));
    public Pose preCollectSpikeMark1Pose = new Pose(42.5, 85, Math.toRadians(180));
    public Pose collectSpikeMark1Pose = new Pose(20, 85, Math.toRadians(180));
    public Pose preCollectSpikeMark3Pose = new Pose(42.5, 35, Math.toRadians(180));
    public Pose collectSpikeMark3Pose = new Pose(20, 35, Math.toRadians(180));

    private final ElapsedTime autoTimer = new ElapsedTime();
    private final ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, startPose, true, Constants.Color.Blue, telemetry, gamepad1);
        autoColor = Constants.Color.Blue;
        robot.drive.resetTeleOpHeading();
        robot.camera.startCapture();
        robot.drive.setConstants(Constants.FConstants);
        robot.intake.setCollectType(Intake.CollectTypes.Normal);
        robot.intake.setState(Intake.States.Wait);

        PathChain preloadPath = robot.drive.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        PathChain goForSpike2 = robot.drive.pathBuilder()
                .addPath(new BezierCurve(
                        shootPose,
                        controlPoint1,
                        preCollectSpikeMark2Pose
                ))
                .setTangentHeadingInterpolation()
                .build();

        PathChain collectSpike2 = robot.drive.pathBuilder()
                .addPath(new BezierLine(preCollectSpikeMark2Pose, collectSpikeMark2Pose))
                .setLinearHeadingInterpolation(preCollectSpikeMark2Pose.getHeading(), collectSpikeMark2Pose.getHeading())
                .build();

        PathChain openGate = robot.drive.pathBuilder()
                .addPath(new BezierLine(collectSpikeMark2Pose, preOpenGatePose))
                .setLinearHeadingInterpolation(collectSpikeMark2Pose.getHeading(), preOpenGatePose.getHeading())
                .build();

        PathChain holdGate = robot.drive.pathBuilder()
                .addPath(new BezierLine(preOpenGatePose, openGatePose))
                .setLinearHeadingInterpolation(preOpenGatePose.getHeading(), openGatePose.getHeading())
                .build();

        PathChain goShoot2 =  robot.drive.pathBuilder()
                .addPath(new BezierCurve(
                        openGatePose,
                        controlPoint2,
                        shootPose
                )).setLinearHeadingInterpolation(openGatePose.getHeading(),shootPose.getHeading())
                .build();

        PathChain goForSpike1 = robot.drive.pathBuilder()
                .addPath(new BezierLine(shootPose, preCollectSpikeMark1Pose))
                .setLinearHeadingInterpolation(collectSpikeMark1Pose.getHeading(), preCollectSpikeMark1Pose.getHeading())
                .build();

        PathChain collectSpike1 = robot.drive.pathBuilder()
                .addPath(new BezierLine(preCollectSpikeMark1Pose, collectSpikeMark1Pose))
                .setLinearHeadingInterpolation(preCollectSpikeMark1Pose.getHeading(), collectSpikeMark1Pose.getHeading())
                .build();

        PathChain goForShoot1 = robot.drive.pathBuilder()
                .addPath(new BezierLine(collectSpikeMark1Pose, shootPose))
                .setLinearHeadingInterpolation(collectSpikeMark1Pose.getHeading(), Math.toRadians(180))
                .build();

        PathChain goForSpike3 = robot.drive.pathBuilder()
                .addPath(new BezierCurve(
                        shootPose,
                        controlPoint3,
                        preCollectSpikeMark3Pose
                )).setTangentHeadingInterpolation()
                .build();

        PathChain collectSpike3 = robot.drive.pathBuilder()
                .addPath(new BezierLine(preCollectSpikeMark3Pose, collectSpikeMark3Pose))
                .setLinearHeadingInterpolation(preCollectSpikeMark3Pose.getHeading(), collectSpikeMark3Pose.getHeading())
                .build();

        PathChain goShootSpike3 = robot.drive.pathBuilder()
                .addPath(new BezierLine(collectSpikeMark3Pose, lastShootPose))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();


        robot.intake.sorter.setColor(Constants.Color.Purple,1);
        robot.intake.sorter.setColor(Constants.Color.Purple,2);
        robot.intake.sorter.setColor(Constants.Color.Purple,3);
        Constants.Globals.afterAuto = true;
        telemetry.addLine("Ready for Action");
        telemetry.update();
        waitForStart();
        autoTimer.reset();
        while (opModeIsActive()) {
            switch (state) {
                case 0:
                    robot.drive.followPath(preloadPath, true);
                    robot.shooter.setTargetVelocity(velocityPreload);
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
                    if(!robot.shootNormal || timer.milliseconds() >= 3000){
                        robot.drive.followPath(goForSpike2, true);
                        state++;
                    }
                    break;
                case 3:
                    if (robot.drive.atParametricEnd()) {
                        robot.intake.setState(Intake.States.Wait);
                        robot.intake.setCollectType(Intake.CollectTypes.Normal);
                        robot.shooter.blocker.setState(BlockerOuttake.States.Open);
                        robot.intake.setPower(IntakeMotor.States.Collect);
                        robot.shooter.setTargetVelocity(velocityNeg);
                        robot.drive.followPath(collectSpike2, true);
                        state++;
                    }
                    break;
                case 4:
                    if (robot.drive.atParametricEnd()) {
                        robot.drive.followPath(openGate, true);
                        state++;
                    }
                    break;
                case 5:
                    if (robot.isDone()) {
                        robot.drive.followPath(holdGate, true);
                        timer.reset();
                        state++;
                    }
                    break;
                case 6:
                    if ((robot.isDone() && timer.milliseconds() >= 1500) || timer.milliseconds() >= 2000) {
                        robot.intake.setPower(IntakeMotor.States.Collect);
                        robot.drive.followPath(goShoot2, true);
                        state++;
                    } else {
                        robot.intake.setPower(IntakeMotor.States.Wait);
                    }
                    break;
                case 7:
                    if (robot.isDone()) {
                        robot.setAction(Robot.Actions.PrepareForShooting);
                        timer.reset();
                        state++;
                    }
                    break;
                case 8:
                    if (timer.milliseconds() >= 200) {
                        robot.shooter.setTargetVelocity(velocityFar);
                        robot.setAction(Robot.Actions.ShootFastNormal);
                        timer.reset();
                        state++;
                    }
                    break;
                case 9:
                    if (!robot.shootNormal || timer.milliseconds() >= 5000) {
                        robot.drive.followPath(goForSpike1, true);
                        robot.shooter.setTargetVelocity(0);
                        state++;
                    }
                    break;
                case 10:
                    if (robot.drive.atParametricEnd()) {
                        robot.intake.setPower(IntakeMotor.States.Collect);
                        robot.shooter.setTargetVelocity(velocityNeg);
                        robot.drive.followPath(collectSpike1, true);
                        state++;
                    }
                    break;
                case 11:
                    if (robot.isDone()) {
                        robot.drive.followPath(goForShoot1, true);
                        state++;
                    }
                    break;
                case 12:
                    if (robot.drive.atParametricEnd()) {
                        robot.drive.turnTo(shootPose.getHeading());
                        state++;
                    }
                    break;
                case 13:
                    if (robot.isDone()) {
                        robot.setAction(Robot.Actions.PrepareForShooting);
                        timer.reset();
                        state = 14;
                    }
                    break;
                case 14:
                    if (timer.milliseconds() >= 200) {
                        robot.shooter.setTargetVelocity(velocityFar);
                        robot.setAction(Robot.Actions.ShootFastNormal);
                        timer.reset();
                        state = 15;
                    }
                    break;
                case 15:
                    if (!robot.shootNormal || timer.milliseconds() >= 5000) {
                        robot.shooter.setTargetVelocity(0);
                        robot.drive.followPath(goForSpike3, true);
                        state++;
                    }
                    break;
                case 16:
                    if (robot.isDone()) {
                        robot.intake.setPower(IntakeMotor.States.Collect);
                        robot.shooter.setTargetVelocity(velocityNeg);
                        robot.drive.followPath(collectSpike3, true);
                        state++;
                    }
                    break;
                case 17:
                    if (robot.isDone()) {
                        robot.drive.followPath(goShootSpike3, true);
                        state++;
                    }
                    break;
                case 18:
                    if (robot.drive.atParametricEnd()) {
                        robot.drive.turnTo(lastShootPose.getHeading());
                        state++;
                    }
                    break;
                case 19:
                    if (robot.isDone()) {
                        robot.setAction(Robot.Actions.PrepareForShooting);
                        timer.reset();
                        state = 20;
                    }
                    break;
                case 20:
                    if (timer.milliseconds() >= 200) {
                        robot.shooter.setTargetVelocity(velocityLast);
                        robot.setAction(Robot.Actions.ShootFastNormal);
                        timer.reset();
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
