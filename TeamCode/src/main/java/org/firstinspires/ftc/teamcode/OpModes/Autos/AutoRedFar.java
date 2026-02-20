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
import org.firstinspires.ftc.teamcode.Core.Module.Intake.IntakeMotor;
import org.firstinspires.ftc.teamcode.Core.Robot;

@Autonomous(name = "🔴AutoFar🔴")
public class AutoRedFar extends LinearOpMode {

    public Robot robot;
    public int state = 0;

    public Pose startPose = new Pose(84, 6, Math.toRadians(90));
    public Pose shootPose = new Pose( 80,12, Math.toRadians(45));
    public Pose collectSpikeMark3Pose = new Pose(126, 34, 0);
    public Pose controlPoint1 = new Pose(80, 38);
    public Pose collectLoadingZone1 = new Pose(128, 7, Math.toRadians(0));
    public Pose preCollectLoadingZone1 = new Pose(113, 7, Math.toRadians(0));

    private final ElapsedTime autoTimer = new ElapsedTime();
    private final ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, startPose, true, Constants.Color.Blue, telemetry, gamepad1);
        robot.outtake.turret.reset();
        autoColor = Constants.Color.Red;
        robot.drive.resetTeleOpHeading();
        robot.camera.startCapture();
        robot.drive.setConstants(Constants.FConstants);

        PathChain preloadPath = robot.drive.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
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
                .setLinearHeadingInterpolation(collectSpikeMark3Pose.getHeading(), Math.toRadians(0))
                .build();

        PathChain collectLoading = robot.drive.pathBuilder()
                .addPath(new BezierLine(shootPose, collectLoadingZone1))
                .setLinearHeadingInterpolation(collectLoadingZone1.getHeading(), collectLoadingZone1.getHeading())
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
                .setLinearHeadingInterpolation(collectLoadingZone1.getHeading(), collectLoadingZone1.getHeading())
                .build();

        Constants.Globals.afterAuto = true;
        robot.shouldAlignTurret = false;
        telemetry.addLine("Ready for Action");
        telemetry.update();
        waitForStart();
        robot.outtake.setShootingVelocity(300);///TODO
        robot.update();
        autoTimer.reset();
        while (opModeIsActive()) {
            switch (state) {
                case 0:
                    robot.drive.followPath(preloadPath);
                    robot.outtake.turret.setTargetDegrees(23);//TODO HABAR NU AM TEO DESCURCA TE
                    state++;
                    break;
                case 1:
                    if(robot.isDone() || autoTimer.milliseconds() > 5000) {
                        robot.setAction(Robot.Actions.Shoot);
                        timer.reset();
                        state = 2;
                    }
                    break;
                case 2:
                    if(!robot.shootingSequence || timer.milliseconds() >= 10000) {
                        robot.drive.followPath(goCollectSpike);
                        robot.intake.setPower(Collect);
                        timer.reset();
                        state = 3;
                    }
                    break;
                case 3:
                    if(robot.isDone()) {
                        robot.outtake.turret.setTargetDegrees(68);//TODO TEO HABAR NU AM 😭
                        robot.drive.followPath(goShootSpike);
                        timer.reset();
                        state++;
                    }
                    break;
                case 4:
                    if (robot.isDone()) {
                        robot.intake.setPower(IntakeMotor.States.Wait);
                        robot.setAction(Robot.Actions.Shoot);
                        timer.reset();
                        state = 5;
                    }
                    break;
                case 5:
                    if(!robot.shootingSequence || timer.milliseconds() >= 4000) {
                        robot.drive.followPath(collectLoading);
                        robot.intake.setPower(Collect);
                        timer.reset();
                        state++;
                    }
                    break;
                case 6:
                    if (robot.isDone()|| robot.intake.isFull) {
                        robot.drive.followPath(preCollectLoading);
                        robot.intake.setPower(Collect);
                        timer.reset();
                        state++;
                    }
                    break;
                case 7:
                    if (robot.isDone() || robot.intake.isFull) {
                        robot.drive.followPath(collectLoading);
                        robot.intake.setPower(Collect);
                        timer.reset();
                        state++;
                    }
                    break;
                case 8:
                    if (timer.milliseconds() >= 750 || robot.intake.isFull) {
                        robot.outtake.turret.setTargetDegrees(62.5);
                        robot.intake.setPower(Collect);
                        if(autoTimer.milliseconds() <= 25000){
                            robot.drive.followPath(goShootLoading);
                            state++;
                        }

                    }
                    break;
                case 9:
                    if (robot.isDone()) {
                        robot.setAction(Robot.Actions.Shoot);
                        timer.reset();
                        if(autoTimer.milliseconds() >= 25000){
                            robot.drive.followPath(preCollectLoading);
                            robot.setAction(Robot.Actions.StopShoot);
                            state++;
                        } else {
                            state = 5;
                        }
                    }
            }

            finalAutoPose = robot.drive.getPose();
            robot.update();
            telemetry.addData("State: ", state);
            telemetry.update();
        }
    }
}