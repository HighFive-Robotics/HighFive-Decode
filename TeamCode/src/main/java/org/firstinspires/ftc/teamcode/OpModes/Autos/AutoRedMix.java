package org.firstinspires.ftc.teamcode.OpModes.Autos;

import static org.firstinspires.ftc.teamcode.Constants.Globals.autoColor;
import static org.firstinspires.ftc.teamcode.Constants.Globals.finalAutoPose;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;

import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Core.Module.Intake.IntakeMotor;
import org.firstinspires.ftc.teamcode.Core.Robot;

@Autonomous(name = "\uD83D\uDD34AutoCloseMix\uD83D\uDD34")
public class  AutoRedMix extends LinearOpMode {

    public Robot robot;
    public int state = 0;

    public Pose startPose = new Pose(125, 114, Math.toRadians(180));//    public Pose startPose = new Pose(13, 113, Math.toRadians(0));

    public Pose shootPose1 = new Pose(93, 95, Math.toRadians(0));
    public Pose shootPose2 = new Pose(83, 83, Math.toRadians(0));
    public Pose shootPose3 = new Pose(85, 110, Math.toRadians(0));

    public Pose preCollectSpikeMark2Pose = new Pose(83, 60, Math.toRadians(0));
    public Pose collectSpikeMark2Pose = new Pose(125.5, 60, Math.toRadians(0));
    public Pose controlPointSpike2 = new Pose(90, 60);

    public Pose collectSpikeMark1Pose = new Pose(120, 83, Math.toRadians(0));

    private final ElapsedTime autoTimer = new ElapsedTime();
    private final ElapsedTime timer = new ElapsedTime();

    public Pose preCollectGatePose = new Pose(119.5, 59, Math.toRadians(40));
    public Pose collectGatePose = new Pose(126, 59, Math.toRadians(40));

    public Pose preCollectLoadingZone2 = new Pose(130, 30, Math.toRadians(-90));
    public Pose loadingControlPoint = new Pose(113, 86, Math.toRadians(-90));
    public Pose collectLoadingZone2 = new Pose(131, 12, Math.toRadians(-80));

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.setMsTransmissionInterval(300);
        robot = new Robot(hardwareMap, startPose, true, Constants.Color.Red, telemetry, gamepad1);
        robot.outtake.turret.reset();
        robot.outtake.startBreakBeamThread();
        autoColor = Constants.Color.Red;
        robot.drive.resetTeleOpHeading();
        robot.camera.startCapture();
        robot.drive.setConstants(Constants.FConstants);
        Constants.Globals.afterAuto = true;
        robot.shouldAlignTurret = false;
        telemetry.setMsTransmissionInterval(500);

        PathChain preloadPath = robot.drive.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose1))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose1.getHeading())
                .build();

        PathChain goForSpike2 = robot.drive.pathBuilder()
                .addPath(new BezierLine(shootPose1, preCollectSpikeMark2Pose))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        PathChain collectSpike2 = robot.drive.pathBuilder()
                .addPath(new BezierLine(preCollectSpikeMark2Pose, collectSpikeMark2Pose))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        PathChain shootSpike2 = robot.drive.pathBuilder()
                .addPath(new BezierCurve(collectSpikeMark2Pose,controlPointSpike2, shootPose2))
                .setLinearHeadingInterpolation(collectSpikeMark2Pose.getHeading(),shootPose2.getHeading())
                .build();

        PathChain collectSpike1 = robot.drive.pathBuilder()
                .addPath(new BezierLine(shootPose2, collectSpikeMark1Pose))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        PathChain shootSpike1 = robot.drive.pathBuilder()
                .addPath(new BezierLine(collectSpikeMark1Pose, shootPose2))
                .setLinearHeadingInterpolation(collectSpikeMark1Pose.getHeading(), shootPose2.getHeading())
                .build();

        PathChain preCollectGate = robot.drive.pathBuilder()
                .addPath(new BezierLine(shootPose2, preCollectGatePose))
                .setLinearHeadingInterpolation(shootPose2.getHeading(),preCollectGatePose.getHeading())
                .build();

        PathChain collectGate = robot.drive.pathBuilder()
                .addPath(new BezierLine(preCollectGatePose, collectGatePose))
                .setLinearHeadingInterpolation(preCollectGatePose.getHeading(),collectGatePose.getHeading())
                .build();

        PathChain shootGate = robot.drive.pathBuilder()
                .addPath(new BezierLine(collectGatePose, shootPose2))
                .setLinearHeadingInterpolation(collectGatePose.getHeading(),shootPose2.getHeading())
                .build();

        PathChain preCollectLoading2 = robot.drive.pathBuilder()
                .addPath(new BezierCurve(shootPose2, loadingControlPoint, preCollectLoadingZone2))
                .setTangentHeadingInterpolation()
                .build();

        PathChain collectLoading2 = robot.drive.pathBuilder()
                .addPath(new BezierLine(preCollectLoadingZone2, collectLoadingZone2))
                .setLinearHeadingInterpolation(preCollectLoadingZone2.getHeading(),collectLoadingZone2.getHeading())
                .build();

        PathChain shootLoading2Final = robot.drive.pathBuilder()
                .addPath(new BezierLine(collectLoadingZone2, shootPose3))
                .setLinearHeadingInterpolation(collectLoadingZone2.getHeading(),shootPose3.getHeading())
                .build();


        telemetry.addLine("Ready for Action");
        telemetry.update();
        robot.update();
        Constants.CameraConstants.toleranceTurretDeg = 0.7;
        Constants.OuttakeConstants.TurretParams.minimumErrorAngleForWalls = Math.PI / 5;
        waitForStart();
        autoTimer.reset();
        robot.outtake.setShootingVelocity(robot.outtake.calculateDistanceToGoal(shootPose1)+2);
        while (opModeIsActive()) {
            switch (state) {
                    case 0:
                        robot.drive.followPath(preloadPath, true);
                        robot.intake.setPower(IntakeMotor.States.Collect);
                        robot.outtake.turret.motor.setMaxPIDPower(0.6);
                        robot.outtake.alignTurret(shootPose1);
                        state= 1;
                        break;
                    case 1:
                        if(robot.isDone()){
                            robot.outtake.setShootingVelocityForPose(shootPose1 , -3);
                            robot.outtake.turret.motor.setMaxPIDPower(1);
                            robot.setAction(Robot.Actions.ResetTurretCamera);
                            timer.reset();
                            state = 2;
                        }
                        break;
                    case 2:
                        if (robot.isDone() && (!robot.resetWithCamera || timer.milliseconds() >= 1150)) {
                            robot.setAction(Robot.Actions.Shoot);
                            state++;
                        }
                        break;
                    case 3:
                        if (!robot.shootingSequence) {
                            robot.drive.followPath(goForSpike2, true);
                            robot.outtake.alignTurret(shootPose2 , -1);
                            robot.outtake.setShootingVelocityForPose(shootPose2 , -2.5);
                            timer.reset();
                            state++;
                        }
                        break;
                    case 4:
                        if (robot.drive.atParametricEnd()) {
                            robot.intake.setPower(IntakeMotor.States.Collect);
                            robot.drive.followPath(collectSpike2, true);
                            state++;
                        }
                        break;
                    case 5:
                        if (robot.drive.atParametricEnd()) {
                            robot.drive.followPath(shootSpike2, true);
                            state++;
                        }
                        break;
                    case 6:
                        if (robot.isDone()) {
                            robot.intake.setPower(IntakeMotor.States.Wait);
                            robot.setAction(Robot.Actions.Shoot);
                            robot.setAction(Robot.Actions.ResetTurretCamera);
                            state++;
                        }
                        break;
                case 7:
                    if (!robot.shootingSequence) {
                        robot.drive.followPath(preCollectGate, true);
                        robot.intake.setPower(IntakeMotor.States.Spit);
                        robot.setAction(Robot.Actions.StopCamera);
                        timer.reset();
                        state++;
                    }
                    break;
                case 8:
                    if (robot.drive.atParametricEnd()) {
                        timer.reset();
                        state++;
                    }
                    break;
                case 9:
                    if (timer.milliseconds() >= 50) {
                        robot.intake.setPower(IntakeMotor.States.Wait);
                        robot.drive.followPath(collectGate, true);
                        timer.reset();
                        state = 10;
                    }
                    break;
                case 10:
                    if (timer.milliseconds() >= 80) {
                        robot.intake.setPower(IntakeMotor.States.Collect);
                        state++;
                    }
                    break;
                case 11:
                    if (robot.isDone() || robot.intake.isPartial) {
                        timer.reset();
                        state++;
                    }
                    break;
                case 12:
                    if (timer.milliseconds() >= 1350) {
                        robot.drive.setMaxPower(1);
                        robot.outtake.setShootingVelocityForPose(shootPose2 , -2.65);
                        robot.drive.followPath(shootGate, true);
                        robot.drive.setMaxPower(1);
                        robot.shouldAlignTurret = true;
                        robot.intake.setPower(IntakeMotor.States.Collect);
                        state++;
                    }
                    break;
                case 13:
                    if (robot.isDone()) {
                        robot.intake.setPower(IntakeMotor.States.Wait);
                        robot.setAction(Robot.Actions.Shoot);
                        state = 14;
                    }
                    break;
                case 14:
                    if (!robot.shootingSequence) {
                        robot.drive.followPath(collectSpike1);
                        robot.intake.setPower(IntakeMotor.States.Collect);
                        robot.setAction(Robot.Actions.StopCamera);
                        timer.reset();
                        state++;
                    }
                    break;
                case 15:
                    if (robot.drive.atParametricEnd()) {
                        robot.intake.setPower(IntakeMotor.States.Collect);
                        robot.drive.followPath(shootSpike1, true);
                        state++;
                    }
                    break;
                case 16:
                    if (robot.isDone()) {
                        robot.intake.setPower(IntakeMotor.States.Wait);
                        robot.setAction(Robot.Actions.Shoot);
                        state = 17;
                    }
                    break;
                case 17:
                    if (!robot.shootingSequence) {
                        robot.shouldAlignTurret = false;
                        robot.drive.followPath(preCollectLoading2, true);
                        robot.intake.setPower(IntakeMotor.States.Collect);
                        robot.setAction(Robot.Actions.StopCamera);
                        timer.reset();
                        state++;
                    }
                    break;
                case 18:
                    if (robot.drive.atParametricEnd()  || robot.intake.isPartial) {
                        robot.drive.followPath(collectLoading2, true);
                        robot.drive.setMaxPower(0.75);
                        timer.reset();
                        state++;
                    }
                    break;
                case 19:
                    if (robot.drive.atParametricEnd() || robot.intake.isPartial || timer.milliseconds() >= 700) {
                        timer.reset();
                        state = 20;
                    }
                    break;
                case 20:
                    if ((timer.milliseconds() >= 450) && autoTimer.milliseconds() <= 28500) {
                        robot.outtake.alignTurret(shootPose3, 1.25);
                        robot.drive.setMaxPower(1);
                        robot.outtake.setShootingVelocityForPose(shootPose3 , -3.3);
                        robot.drive.followPath(shootLoading2Final, true);
                        timer.reset();
                        state++;
                    }
                    break;
                case 21:
                    if (robot.isDone()) {
                        robot.intake.setPower(IntakeMotor.States.Wait);
                        robot.setAction(Robot.Actions.Shoot);
                        timer.reset();
                        state++;
                    }
                    break;
            }

            if(state == 3){
                robot.outtake.alignTurret(shootPose1);
            }
            if(state == 7){
                robot.outtake.alignTurret(shootPose2);
            }
            if(state == 14){
                robot.outtake.alignTurret(shootPose2);
            }
            if(state == 17){
                robot.outtake.alignTurret(shootPose2);
            }
            if(state == 22){
                robot.outtake.alignTurret(shootPose3);
            }

            finalAutoPose = robot.drive.getPose();
            robot.update();

            telemetry.addData("State", state);
            telemetry.addData("Timer", autoTimer.seconds());
            telemetry.addData("Intake Full", robot.intake.isFull);
            telemetry.addData("Shooting Seq Active", robot.shootingSequence);
            telemetry.update();
        }

        robot.outtake.stopBreakBeamThread();
        Constants.CameraConstants.toleranceTurretDeg = 1;
        Constants.OuttakeConstants.TurretParams.minimumErrorAngleForWalls = Math.PI / 10;
    }
}