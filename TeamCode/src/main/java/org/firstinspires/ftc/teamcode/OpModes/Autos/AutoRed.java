package org.firstinspires.ftc.teamcode.OpModes.Autos;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
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
import org.firstinspires.ftc.teamcode.Core.Robot;

@Autonomous(name = "🔴AutoClose🔴")
public class AutoRed extends LinearOpMode {

    public Robot robot;
    public int state = 0;

    public Pose startPose = new Pose(130, 113, Math.toRadians(180));
    public Pose shootPose = new Pose(87, 85, Math.toRadians(-130));
    public Pose lastShootPose = new Pose(92, 114.5, Math.toRadians(-150));
    public Pose preOpenGatePose = new Pose(123, 68, Math.toRadians(-90));
    public Pose openGatePose = new Pose(128.5, 68, Math.toRadians(-90));
    public Pose controlPoint1 = new Pose(84, 62);
    public Pose controlPoint2 = new Pose(95, 67);
    public Pose controlPoint3 = new Pose(60, 42);
    public Pose controlPoint4 = new Pose(130, 58);
    public Pose preCollectSpikeMark2Pose = new Pose(99.5, 58,0);
    public Pose collectSpikeMark2Pose = new Pose(125, 58, 0);
    public Pose preCollectSpikeMark1Pose = new Pose(99.5, 85, 0);
    public Pose collectSpikeMark1Pose = new Pose(122, 85, 0);
    public Pose preCollectSpikeMark3Pose = new Pose(99.5, 37, 0);
    public Pose collectSpikeMark3Pose = new Pose(126, 37, 0);
    public Pose collectLoadingZone1 = new Pose(130, 12, Math.toRadians(-90));
    public Pose preCollectLoadingZone1 = new Pose(130, 22, Math.toRadians(-90));

    private final ElapsedTime autoTimer = new ElapsedTime();
    private final ElapsedTime timer = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, startPose, true, Constants.Color.Red, telemetry, gamepad1);
        robot.outtake.turret.reset();
        autoColor = Constants.Color.Red;
        robot.drive.resetTeleOpHeading();
        robot.camera.startCapture();
        robot.drive.setConstants(Constants.FConstants);

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
                .addPath(new BezierLine(shootPose, collectSpikeMark1Pose))
                .setLinearHeadingInterpolation(collectSpikeMark1Pose.getHeading(), collectSpikeMark1Pose.getHeading())
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
                .addPath(new BezierLine(collectSpikeMark3Pose, shootPose))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        PathChain goCollectLoadingZone1 = robot.drive.pathBuilder()
                .addPath(new BezierCurve(
                        shootPose,
                        controlPoint4,
                        collectLoadingZone1
                )).setTangentHeadingInterpolation()
                .build();

        PathChain loading1 = robot.drive.pathBuilder()
                .addPath(new BezierLine(collectLoadingZone1, preCollectLoadingZone1))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        PathChain loading2 = robot.drive.pathBuilder()
                .addPath(new BezierLine(preCollectLoadingZone1, collectLoadingZone1))
                .setTangentHeadingInterpolation()
                .build();

        PathChain goShootLast = robot.drive.pathBuilder()
                .addPath(new BezierCurve(
                        collectLoadingZone1,
                        controlPoint4,
                        lastShootPose
                )).setTangentHeadingInterpolation()
                .setReversed()
                .build();

        Constants.Globals.afterAuto = true;
        robot.shouldAlignTurret = false;
        telemetry.addLine("Ready for Action");
        telemetry.update();
        waitForStart();
        robot.outtake.setShootingVelocity(150);
        robot.update();
        autoTimer.reset();
        while (opModeIsActive()) {
            switch (state) {
                case 0:
                    robot.drive.followPath(preloadPath, true);
                    robot.intake.setPower(IntakeMotor.States.Collect);
                    robot.outtake.turret.setTargetDegrees(-167);
                    state++;
                    break;
                case 1:
                    if(robot.isDone() || autoTimer.milliseconds() > 5000) {
                        robot.setAction(Robot.Actions.Shoot);
                        timer.reset();
                        state = 100;
                    }
                    break;
                case 100:
                    if(!robot.shootingSequence || timer.milliseconds() >= 4000) {
                        robot.drive.turnTo(preCollectSpikeMark1Pose.getHeading());
                        timer.reset();
                        state = 2;
                    }
                    break;
                case 2:
                    if(robot.isDone()){
                        robot.intake.setPower(IntakeMotor.States.Collect);
                        robot.outtake.closeBlocker();
                        robot.outtake.setShootingVelocity(130);
                        robot.outtake.turret.setTargetDegrees(45);
                        robot.drive.followPath(goForSpike1, true);
                        state = 3;
                    }
                    break;
                case 3:
                    if(robot.drive.atParametricEnd()){
                        robot.drive.followPath(collectSpike1, true);
                        state = 5;
                    }
                    break;
                case 5:
                    if (robot.isDone() || robot.intake.isFull) {
                        robot.drive.followPath(goForShoot1, true);
                        state = 6;
                    }
                    break;
                case 6:
                    if (robot.isDone()) {
                        robot.intake.setPower(IntakeMotor.States.Wait);
                        robot.setAction(Robot.Actions.Shoot);
                        timer.reset();
                        state = 7;
                    }
                    break;
                case 7:
                    if (!robot.shootingSequence || timer.milliseconds() >= 5000) {
                        robot.drive.followPath(goForSpike2, true);
                        robot.outtake.turret.setTargetDegrees(-173);
                        state = 700;
                    }
                    break;
                case 700:
                    if (robot.drive.atParametricEnd()) {
                        robot.intake.setPower(IntakeMotor.States.Collect);
                        robot.outtake.closeBlocker();
                        robot.drive.followPath(collectSpike2, true);
                        state = 8;
                    }
                    break;
                case 8:
                    if (robot.drive.atParametricEnd()) {
                        robot.drive.followPath(openGate, true);
                        state++;
                    }
                    break;
                case 9:
                    if (robot.isDone()) {
                        robot.drive.followPath(holdGate, true);
                        timer.reset();
                        state++;
                    }
                    break;
                case 10:
                    if ((robot.isDone() && timer.milliseconds() >= 1500) || timer.milliseconds() >= 1500) {
                        robot.intake.setPower(IntakeMotor.States.Collect);
                        robot.drive.followPath(goShoot2, true);
                        state++;
                    } else {
                        robot.intake.setPower(IntakeMotor.States.Wait);
                    }
                    break;
                case 11:
                    if(robot.isDone()){
                        robot.setAction(Robot.Actions.Shoot);
                        timer.reset();
                        state++;
                    }
                    break;
                case 12:
                    if (!robot.shootingSequence || timer.milliseconds() >= 5000) {
                        robot.outtake.turret.setTargetDegrees(100);
                        robot.intake.setPower(IntakeMotor.States.Collect);
                        robot.drive.followPath(goForSpike3, true);
                        state++;
                    }
                    break;
                case 13:
                    if (robot.drive.atParametricEnd()) {
                        robot.intake.setPower(IntakeMotor.States.Collect);
                        robot.drive.setMaxPower(0.8);
                        robot.drive.followPath(collectSpike3, true);
                        state++;
                    }
                    break;
                case 14:
                    if (robot.isDone()) {
                        robot.drive.setMaxPower(1);
                        robot.drive.followPath(goShootSpike3, true);
                        state = 15;
                    }
                    break;
                case 15:
                    if (robot.isDone()) {
                        robot.setAction(Robot.Actions.Shoot);
                        timer.reset();
                        state++;
                    }
                    break;
                case 16:
                    if (!robot.shootingSequence || timer.milliseconds() >= 5000) {
                        robot.outtake.turret.setTargetDegrees(0);
                        robot.outtake.setShootingVelocity(90);
                        robot.intake.setPower(IntakeMotor.States.Collect);
                        robot.drive.followPath(goCollectLoadingZone1, true);
                        state++;
                    }
                    break;
                case 17:
                    if (robot.isDone()|| robot.intake.isFull) {
                        robot.drive.followPath(loading1);
                        timer.reset();
                        state++;
                    }
                    break;
                case 18:
                    if (robot.isDone() || robot.intake.isFull) {
                        robot.drive.followPath(loading2);
                        timer.reset();
                        state++;
                    }
                    break;
                case 19:
                    if (timer.milliseconds() >= 750 || robot.intake.isFull) {
                        robot.outtake.turret.setTargetDegrees(85);
                        robot.drive.followPath(goShootLast);
                        state++;
                    }
                    break;
                case 20:
                    if (robot.isDone()) {
                        robot.setAction(Robot.Actions.Shoot);
                        timer.reset();
                        state++;
                    }
            }

            finalAutoPose = robot.drive.getPose();
            robot.update();
            telemetry.addData("State: ", state);
            telemetry.update();
        }
    }
}