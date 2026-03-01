package org.firstinspires.ftc.teamcode.OpModes.AutoFar;

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

@Autonomous(name = "🔴AutoFar🔴")
public class AutoRedFarDetection extends LinearOpMode {

    public Robot robot;
    public int state = 0;


    public Pose startPose = new Pose(84.5, 6, Math.toRadians(0)); //public Pose startPose = new Pose(50, 6, Math.toRadians(180));
    public Pose precollectSpikeMark3Pose = new Pose(84.5, 35, Math.toRadians(0));
    public Pose controlPointLoading1 = new Pose(25.5, 11).mirror();
    public Pose controlPointLoading2 = new Pose(10.5, 21).mirror();
    public Pose collectSpikeMark3Pose = new Pose(126, 35, Math.toRadians(0));
    public Pose collectLoadingZone1 = new Pose(127.5, 6, Math.toRadians(0));
    public Pose preCollectLoadingZone1 = new Pose(108.5, 6, Math.toRadians(0));

    public Pose preCollectLoadingZone2 = new Pose(127.5, 17, Math.toRadians(-30));
    public Pose collectLoadingZone2 = new Pose(129, 8, Math.toRadians(-30));

    public Pose parkPose = new Pose(127.5, 10, Math.toRadians(0));

    public Pose loadingArtifact = new Pose(127.5, 10, Math.toRadians(-90));

    private final ElapsedTime autoTimer = new ElapsedTime();
    private final ElapsedTime timer = new ElapsedTime();
    private final ElapsedTime scanTimer = new ElapsedTime();
    private double scanAngle = 0;
    private int scanDirection = 1;
    private Pose cameraPose;
    private PathChain ballPath;
    PathChain auxToLoading;
    Boolean isFirstTime = true;


    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, startPose, true, Constants.Color.Red, telemetry, gamepad1);
        telemetry.setMsTransmissionInterval(1200);
        robot.outtake.turret.reset();
        autoColor = Constants.Color.Red;
        robot.drive.resetTeleOpHeading();
        robot.camera.setPipeline(HighCamera.Pipelines.AprilTagLocation);
        robot.camera.startCapture();
        robot.drive.setConstants(Constants.FConstants);

        PathChain collectLoading2 = robot.drive.pathBuilder()
                .addPath(new BezierLine(startPose, preCollectLoadingZone2))
                .setLinearHeadingInterpolation(startPose.getHeading(), preCollectLoadingZone2.getHeading())
                .addPath(new BezierLine(preCollectLoadingZone2, collectLoadingZone2))
                .setLinearHeadingInterpolation(preCollectLoadingZone2.getHeading(), collectLoadingZone2.getHeading())
                .build();

        PathChain collectLoading = robot.drive.pathBuilder()
                .addPath(new BezierCurve(startPose,
                        controlPointLoading1,
                        controlPointLoading2,
                        collectLoadingZone1))
                .setLinearHeadingInterpolation(startPose.getHeading(), collectLoadingZone1.getHeading())
                .build();
        PathChain preCollectLoading = robot.drive.pathBuilder()
                .addPath(new BezierLine(collectLoadingZone1, preCollectLoadingZone1))
                .setLinearHeadingInterpolation(collectLoadingZone1.getHeading(), preCollectLoadingZone1.getHeading())
                .build();
        PathChain finishCollecting = robot.drive.pathBuilder()
                .addPath(new BezierLine(preCollectLoadingZone1, collectLoadingZone1))
                .setLinearHeadingInterpolation(preCollectLoadingZone1.getHeading(), collectLoadingZone1.getHeading())
                .build();
        PathChain CollectLoadingZone = robot.drive.pathBuilder()
                .addPath(new BezierLine(collectLoadingZone2, preCollectLoadingZone2))
                .setLinearHeadingInterpolation(collectLoadingZone2.getHeading(), preCollectLoadingZone2.getHeading())
                .build();
        PathChain goShootLoading = robot.drive.pathBuilder()
                .addPath(new BezierLine(collectLoadingZone1, startPose))
                .setLinearHeadingInterpolation(collectLoadingZone1.getHeading(), startPose.getHeading())
                .build();

        PathChain preCollectSpike3 = robot.drive.pathBuilder()
                .addPath(new BezierLine(startPose, precollectSpikeMark3Pose))
                .setLinearHeadingInterpolation(startPose.getHeading(), precollectSpikeMark3Pose.getHeading())
                .build();

        PathChain collectSpike3 = robot.drive.pathBuilder()
                .addPath(new BezierLine(
                        precollectSpikeMark3Pose,
                        collectSpikeMark3Pose
                ))
                .setLinearHeadingInterpolation(precollectSpikeMark3Pose.getHeading(),collectSpikeMark3Pose.getHeading())
                .build();

        PathChain goShootSpike = robot.drive.pathBuilder()
                .addPath(new BezierLine(collectSpikeMark3Pose, startPose))
                .setLinearHeadingInterpolation(collectSpikeMark3Pose.getHeading(), Math.toRadians(0))
                .build();

        PathChain park = robot.drive.pathBuilder()
                .addPath(new BezierLine(startPose, parkPose))
                .setLinearHeadingInterpolation(startPose.getHeading(),parkPose.getHeading())
                .build();

        PathChain goShootDetection = robot.drive.pathBuilder()
                .addPath(new BezierLine(loadingArtifact, startPose))
                .setLinearHeadingInterpolation(loadingArtifact.getHeading(), startPose.getHeading())
                .build();

        Constants.Globals.afterAuto = true;
        robot.shouldAlignTurret = false;
        telemetry.addLine("Ready for Action");
        telemetry.update();
        waitForStart();

        robot.outtake.setShootingVelocityForPose(startPose, -1);
        robot.update();
        autoTimer.reset();
        timer.reset();

        while (opModeIsActive()) {
            if (autoTimer.milliseconds() >= 27500 && state < 100 &&  !robot.intake.isPartial) {
                state = 100;
            }

            switch (state) {
                case 0:
                    robot.shouldAlignTurret = true;
                    robot.setAction(Robot.Actions.ResetTurretCamera);
                    state++;
                    break;
                case 1:
                    if (!robot.resetWithCamera) {
                        robot.outtake.turret.addOffsetDegrees(-1.5);
                        robot.setAction(Robot.Actions.Shoot);
                        timer.reset();
                        state = 2;
                    }
                    break;
                case 2:
                    if (!robot.shootingSequence) {
                        robot.drive.followPath(collectLoading2);
                        robot.shouldAlignTurret = false;
                        robot.intake.setPower(Collect);
                        timer.reset();
                        state = 4;
                    }
                    break;
//                case 3:
//                    if (robot.isDone()) {
//                        robot.drive.followPath(preCollectLoading);
//                        robot.intake.setPower(Collect);
//                        timer.reset();
//                        state++;
//                    }
//                    break;
                case 4:
                    if (robot.isDone()) {
                        robot.intake.setPower(Collect);
                        robot.outtake.setShootingVelocityForPose(startPose, -7.5);
                        timer.reset();
                        state++;
                    }
                    break;
                case 5:
                    if (timer.milliseconds() >= 650 || robot.intake.isFull) {
                        robot.shouldAlignTurret = true;
                        robot.drive.followPath(goShootLoading);
                        timer.reset();
                        state = 6;
                    }
                    break;
//                case 50:
//                    if (robot.isDone()) {
//                        robot.setAction(Robot.Actions.ResetTurretCamera);
//                        state = 6;
//                    }
//                    break;
                case 55:
                    if (robot.isDone()) {
                        timer.reset();
                        state = 6;
                    }
                    break;
                case 6:
                    if (robot.isDone() && timer.milliseconds() >= 400) {
                        robot.intake.setPower(IntakeMotor.States.Wait);
                        robot.setAction(Robot.Actions.Shoot);
                        timer.reset();
                        state++;
                    }
                    break;
                case 7:
                    if (!robot.shootingSequence) {
                        if(!isFirstTime){
                            state = 2;
                        }
                        robot.drive.followPath(preCollectSpike3);
                        robot.shouldAlignTurret = false;
                        robot.intake.setPower(Collect);
                        timer.reset();
                        state = 75;
                    }
                    break;
                case 75:
                    if (robot.drive.atParametricEnd()) {
                        robot.intake.setPower(IntakeMotor.States.Collect);
                        robot.drive.followPath(collectSpike3, true);
                        state = 8;
                    }
                    break;
                case 8:
                    if (robot.isDone()) {
                        robot.drive.followPath(goShootSpike);
                        robot.outtake.setShootingVelocityForPose(startPose, -8.5);
                        timer.reset();
                        state = 85;
                    }
                    break;
//                case 9:
//                    if (robot.isDone() || timer.milliseconds() >= 5000) {
//                        robot.setAction(Robot.Actions.ResetTurretCamera);
//                        timer.reset();
//                        state = 90;
//                    }
//                    break;
                case 85:
                    if (robot.isDone()) {
                        timer.reset();
                        state = 90;
                    }
                    break;
                case 90:
                    if (robot.isDone() && timer.milliseconds() >= 400) {
                        robot.intake.setPower(IntakeMotor.States.Wait);
                        robot.setAction(Robot.Actions.Shoot);
                        timer.reset();
                        state = 2;
                    }
                    break;



//                case 10:
//                    if (!robot.shootingSequence || timer.milliseconds() >= 3000) {
//                        scanAngle = 0;
//                        scanDirection = 1;
//                        robot.outtake.linkageCamera.setState(LinkageCamera.States.Artifact, 300);
//                        robot.camera.setPipeline(HighCamera.Pipelines.BallDetection);
//                        robot.shouldAlignTurret = false;
//                        robot.outtake.turret.setOffset(0);
//                        robot.intake.setPower(Collect);
//                        robot.outtake.turret.setTarget(scanAngle);
//                        scanTimer.reset();
//                        timer.reset();
//                        state++;
//                    }
//                    break;
//                case 11:
//                    cameraPose = robot.camera.getBallPose(robot.drive.getPose());
//                    if (cameraPose != null) {
//                        ballPath = robot.drive.pathBuilder()
//                                .addPath(new BezierLine(
//                                        robot.drive.getPose(),
//                                        cameraPose
//                                ))
//                                .setTangentHeadingInterpolation()
//                                .build();
//                        robot.drive.followPath(ballPath);
//                        state++;
//                    } else {
//                        if (scanTimer.milliseconds() >= 150) {
//                            scanAngle += 5 * scanDirection;
//                            if(scanAngle >= 60){
//                                scanDirection = -1;
//                            }else if (scanAngle <= -60){
//                                scanDirection = 1;
//                            }
//                            scanTimer.reset();
//                        }
//                    }
//                    break;
//                case 12:
//                    if(robot.isDone()){
//                        auxToLoading = robot.drive.pathBuilder()
//                                .addPath(new BezierLine(
//                                        robot.drive.getPose(),
//                                        loadingArtifact
//                                ))
//                                .setLinearHeadingInterpolation(robot.drive.getHeading() , loadingArtifact.getHeading())
//                                .build();
//                        robot.drive.followPath(auxToLoading);
//                        state++;
//                        robot.outtake.setShootingVelocityForPose(startPose, -8.5);
//                    }
//                    break;
//                case 13:
//                    if(robot.isDone()){
//                        robot.drive.followPath(ballPath,true);
//                        robot.outtake.setShootingVelocityForPose(startPose, -8.5);
//                        state = 9;
//                    }
//                    break;
                case 100:
                    robot.outtake.linkageCamera.setState(LinkageCamera.States.Goal);
                    robot.intake.setPower(IntakeMotor.States.Wait);
                    robot.setAction(Robot.Actions.StopShoot);
                    robot.drive.followPath(park);
                    state++;
                    break;
                case 101:
                    break;
            }
            finalAutoPose = robot.drive.getPose();
            robot.update();
            telemetry.addData("State: ", state);
            telemetry.update();
        }

        robot.outtake.stopBreakBeamThread();
    }
}