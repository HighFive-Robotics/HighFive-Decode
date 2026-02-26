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
import org.firstinspires.ftc.teamcode.Core.Hardware.HighCamera;
import org.firstinspires.ftc.teamcode.Core.Module.Intake.IntakeMotor;
import org.firstinspires.ftc.teamcode.Core.Module.Outtake.LinkageCamera;
import org.firstinspires.ftc.teamcode.Core.Robot;

@Autonomous(name = "🔵AutoFar Detectie🔵")
public class AutoBlueFarDetection extends LinearOpMode {

    public Robot robot;
    public int state = 0;

    public Pose startPose = new Pose(54, 6, Math.toRadians(90));
    public Pose shootPose = new Pose( 56,12, Math.toRadians(135));
    public Pose collectSpikeMark3Pose = new Pose(16, 34, Math.toRadians(180));
    public Pose controlPoint1 = new Pose(60, 38);
    public Pose collectLoadingZone1 = new Pose(12, 12, Math.toRadians(180));
    public Pose preCollectLoadingZone1 = new Pose(25, 12, Math.toRadians(180));
    public Pose parkPose = new Pose(12, 12, Math.toRadians(180));

    private final ElapsedTime autoTimer = new ElapsedTime();
    private final ElapsedTime timer = new ElapsedTime();
    private final ElapsedTime scanTimer = new ElapsedTime();
    private int collectedArtifactsCount = 0;
    private double scanAngle = 0;
    private int scanDirection = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, startPose, true, Constants.Color.Blue, telemetry, gamepad1);
        robot.outtake.turret.reset();
        autoColor = Constants.Color.Blue;
        robot.drive.resetTeleOpHeading();
        robot.camera.startCapture();
        robot.drive.setConstants(Constants.FConstants);
        PathChain preloadPath = robot.drive.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();
        PathChain collectLoading = robot.drive.pathBuilder()
                .addPath(new BezierLine(shootPose, collectLoadingZone1))
                .setLinearHeadingInterpolation(shootPose.getHeading(), collectLoadingZone1.getHeading())
                .build();

        PathChain preCollectLoading = robot.drive.pathBuilder()
                .addPath(new BezierLine(collectLoadingZone1, preCollectLoadingZone1))
                .setLinearHeadingInterpolation(collectLoadingZone1.getHeading(), preCollectLoadingZone1.getHeading())
                .build();

        PathChain finishCollecting = robot.drive.pathBuilder()
                .addPath(new BezierLine(preCollectLoadingZone1, collectLoadingZone1))
                .setLinearHeadingInterpolation(preCollectLoadingZone1.getHeading(), collectLoadingZone1.getHeading())
                .build();

        PathChain goShootLoading = robot.drive.pathBuilder()
                .addPath(new BezierLine(collectLoadingZone1, shootPose))
                .setLinearHeadingInterpolation(collectLoadingZone1.getHeading(), shootPose.getHeading())
                .build();
        PathChain goCollectSpike = robot.drive.pathBuilder()
                .addPath(new BezierCurve(
                        shootPose,
                        controlPoint1,
                        collectSpikeMark3Pose
                )).setTangentHeadingInterpolation()
                .build();

        PathChain goShootSpike = robot.drive.pathBuilder()
                .addPath(new BezierLine(collectSpikeMark3Pose, shootPose))
                .setLinearHeadingInterpolation(collectSpikeMark3Pose.getHeading(), Math.toRadians(180))
                .build();

        Constants.Globals.afterAuto = true;
        robot.shouldAlignTurret = false;
        telemetry.addLine("Ready for Action");
        telemetry.update();
        waitForStart();

        robot.outtake.setShootingVelocityForPose(shootPose);
        robot.update();
        autoTimer.reset();
        timer.reset();

        while (opModeIsActive()) {
            if (autoTimer.milliseconds() >= 27500 && state < 100) {
                state = 100;
            }

            switch (state) {
                case 0:
                    robot.drive.followPath(preloadPath);
                    robot.outtake.alignTurret(shootPose);
                    state++;
                    break;
                case 1:
                    if(robot.isDone() || autoTimer.milliseconds() > 5000) {
                        robot.setAction(Robot.Actions.Shoot);
                        timer.reset();
                        state++;
                    }
                    break;
                case 2:
                    if(!robot.shootingSequence || timer.milliseconds() >= 3000) {
                        robot.drive.followPath(collectLoading);
                        robot.intake.setPower(Collect);
                        timer.reset();
                        state++;
                    }
                    break;
                case 3:
                    if (robot.isDone() || robot.intake.isFull) {
                        robot.drive.followPath(preCollectLoading);
                        timer.reset();
                        state++;
                    }
                    break;
                case 4:
                    if (robot.isDone() || robot.intake.isFull) {
                        robot.drive.followPath(finishCollecting);
                        timer.reset();
                        state++;
                    }
                    break;
                case 5:
                    if (timer.milliseconds() >= 750 || robot.intake.isFull) {
                        robot.outtake.alignTurret(shootPose);
                        robot.drive.followPath(goShootLoading);
                        timer.reset();
                        state++;
                    }
                    break;
                case 6:
                    if (robot.isDone() || timer.milliseconds() >= 5000) {
                        robot.intake.setPower(IntakeMotor.States.Wait);
                        robot.setAction(Robot.Actions.Shoot);
                        timer.reset();
                        state++;
                    }
                    break;
                case 7:
                    if(!robot.shootingSequence || timer.milliseconds() >= 3000) {
                        robot.drive.followPath(goCollectSpike);
                        robot.intake.setPower(Collect);
                        timer.reset();
                        state++;
                    }
                    break;
                case 8:
                    if(robot.isDone() || robot.intake.isFull) {
                        robot.outtake.alignTurret(shootPose);
                        robot.drive.followPath(goShootSpike);
                        timer.reset();
                        state++;
                    }
                    break;
                case 9:
                    if (robot.isDone() || timer.milliseconds() >= 5000) {
                        robot.intake.setPower(IntakeMotor.States.Wait);
                        robot.setAction(Robot.Actions.Shoot);
                        timer.reset();
                        state++;
                    }
                    break;
                case 10:
                    if(!robot.shootingSequence || timer.milliseconds() >= 3000) {
                        robot.outtake.linkageCamera.setState(LinkageCamera.States.Artifact, 300);
                        robot.camera.setPipeline(HighCamera.Pipelines.BallDetection);
                        scanAngle = 0;
                        scanDirection = 1;
                        scanTimer.reset();
                        state++;
                    }
                    break;
                case 11:
                    Pose camPose = robot.camera.getBallPose(robot.drive.getPose());
                    boolean isValidDetection = camPose != null &&
                            Math.hypot(camPose.getX() - robot.drive.getPose().getX(), camPose.getY() - robot.drive.getPose().getY()) > 2.0;
                    if (isValidDetection) {
                        PathChain targetPath = robot.drive.pathBuilder()
                                .addPath(new BezierLine(robot.drive.getPose(), camPose))
                                .setLinearHeadingInterpolation(robot.drive.getPose().getHeading(), camPose.getHeading())
                                .build();

                        robot.drive.followPath(targetPath);
                        robot.intake.setPower(Collect);
                        timer.reset();
                        state++;
                    } else {
                        if (scanTimer.milliseconds() > 70) {
                            scanAngle += (2.0 * scanDirection);
                            if (scanAngle > 60 || scanAngle < -60) {
                                scanDirection *= -1;
                            }
                            robot.outtake.turret.setTargetDegrees(scanAngle);
                            scanTimer.reset();
                        }
                    }
                    break;
                case 12:
                    if (robot.isDone() || robot.intake.isFull || timer.milliseconds() > 4000) {
                        if (robot.intake.isFull) {
                            state = 14;
                        } else {
                            Pose currentPose = robot.drive.getPose();
                            Pose pushForward = new Pose(
                                    currentPose.getX() + 15 * Math.cos(currentPose.getHeading()),
                                    currentPose.getY() + 15 * Math.sin(currentPose.getHeading()),
                                    currentPose.getHeading()
                            );

                            PathChain pushPath = robot.drive.pathBuilder()
                                    .addPath(new BezierLine(currentPose, pushForward))
                                    .setLinearHeadingInterpolation(currentPose.getHeading(), pushForward.getHeading())
                                    .build();

                            robot.drive.followPath(pushPath);
                            timer.reset();
                            state++;
                        }
                    }
                    break;
                case 13:
                    if (robot.isDone() || robot.intake.isFull || timer.milliseconds() > 2500) {
                        state++;
                    }
                    break;
                case 14:
                    PathChain backToShoot = robot.drive.pathBuilder()
                            .addPath(new BezierLine(robot.drive.getPose(), shootPose))
                            .setLinearHeadingInterpolation(robot.drive.getPose().getHeading(), shootPose.getHeading())
                            .build();

                    robot.drive.followPath(backToShoot);
                    robot.intake.setPower(IntakeMotor.States.Wait);
                    robot.outtake.linkageCamera.setState(LinkageCamera.States.Goal, 300);
                    timer.reset();
                    state++;
                    break;
                case 15:
                    robot.outtake.alignTurret(robot.drive.getPose());
                    if (robot.isDone() || timer.milliseconds() > 4000) {
                        robot.setAction(Robot.Actions.Shoot);
                        collectedArtifactsCount++;
                        timer.reset();
                        state++;
                    }
                    break;
                case 16:
                    if(!robot.shootingSequence || timer.milliseconds() >= 3000) {
                        state = 10;
                    }
                    break;
                case 100:
                    robot.outtake.linkageCamera.setState(LinkageCamera.States.Goal);
                    robot.intake.setPower(IntakeMotor.States.Wait);

                    PathChain parkingPath = robot.drive.pathBuilder()
                            .addPath(new BezierLine(robot.drive.getPose(), parkPose))
                            .setLinearHeadingInterpolation(robot.drive.getPose().getHeading(), parkPose.getHeading())
                            .build();
                    robot.drive.followPath(parkingPath);
                    state++;
                    break;
                case 101:
                    break;
            }

            finalAutoPose = robot.drive.getPose();
            robot.update();

            telemetry.addData("State: ", state);
            telemetry.addData("Cycles Performed: ", collectedArtifactsCount);
            telemetry.addData("Time Left: ", 30 - (autoTimer.milliseconds() / 1000.0));
            telemetry.update();
        }

        robot.outtake.stopBreakBeamThread();
    }
}