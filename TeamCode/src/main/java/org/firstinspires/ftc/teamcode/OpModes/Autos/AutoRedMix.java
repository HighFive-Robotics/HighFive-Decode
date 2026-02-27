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

@Autonomous(name = "🔵AutoCloseMix🔵")
public class  AutoRedMix extends LinearOpMode {

    public Robot robot;
    public int state = 0;

    public Pose startPose = new Pose(125, 114, Math.toRadians(180));//    public Pose startPose = new Pose(13, 113, Math.toRadians(0));

    public Pose shootPose1 = new Pose(93, 95, Math.toRadians(0));
    public Pose shootPose2 = new Pose(83, 81.5, Math.toRadians(0));
    public Pose shootPose3 = new Pose(78, 110, Math.toRadians(-90));

    public Pose preCollectSpikeMark2Pose = new Pose(83, 60, Math.toRadians(0));
    public Pose collectSpikeMark2Pose = new Pose(124, 60, Math.toRadians(0));
    public Pose controlPointSpike2 = new Pose(48.5, 60);

    public Pose collectSpikeMark1Pose = new Pose(120, 81.5, Math.toRadians(0));
    public Pose preOpenGatePose = new Pose(117, 70, Math.toRadians(-90));
    public Pose openGatePose = new Pose(123, 70, Math.toRadians(-90));
    public Pose controlPointGate = new Pose(85, 67.5);

    public Pose preCollectGatePose = new Pose(119.5, 58, Math.toRadians(0));
    public Pose collectGatePose = new Pose(126, 58, Math.toRadians(0));

    public Pose preCollectLoadingZone2 = new Pose(131, 50, Math.toRadians(-90));
    public Pose collectLoadingZone2 = new Pose(131, 12, Math.toRadians(-90));

    private final ElapsedTime autoTimer = new ElapsedTime();
    private final ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.setMsTransmissionInterval(300);
        robot = new Robot(hardwareMap, startPose, true, Constants.Color.Blue, telemetry, gamepad1);
        robot.outtake.turret.reset();
        robot.outtake.startBreakBeamThread();
        autoColor = Constants.Color.Blue;
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
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        PathChain collectSpike1 = robot.drive.pathBuilder()
                .addPath(new BezierLine(shootPose2, collectSpikeMark1Pose))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        PathChain preOpenGate = robot.drive.pathBuilder()
                .addPath(new BezierLine(collectSpikeMark1Pose, preOpenGatePose))
                .setLinearHeadingInterpolation(collectSpikeMark1Pose.getHeading(), preOpenGatePose.getHeading())
                .build();

        PathChain openGate = robot.drive.pathBuilder()
                .addPath(new BezierLine(preOpenGatePose, openGatePose))
                .setLinearHeadingInterpolation(preOpenGatePose.getHeading(), openGatePose.getHeading())
                .build();

        PathChain shootSpike1 = robot.drive.pathBuilder()
                .addPath(new BezierCurve(openGatePose, controlPointGate, shootPose2))
                .setLinearHeadingInterpolation(openGatePose.getHeading(), shootPose2.getHeading())
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
                .addPath(new BezierLine(shootPose2, preCollectLoadingZone2))
                .setLinearHeadingInterpolation(preCollectLoadingZone2.getHeading(),preCollectLoadingZone2.getHeading())
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
                        robot.outtake.setShootingVelocityForPose(shootPose1);
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
                        robot.outtake.alignTurret(shootPose2 , 2);
                        robot.outtake.setShootingVelocity(robot.outtake.calculateDistanceToGoal(shootPose2)-2);
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
                        robot.intake.setPower(IntakeMotor.States.Collect);
                        robot.drive.followPath(shootSpike2, true);
                        state++;
                    }
                    break;
                case 6:
                    if (robot.isDone()) {
                        robot.intake.setPower(IntakeMotor.States.Wait);
                        robot.setAction(Robot.Actions.Shoot);
                        state++;
                    }
                    break;
                case 7:
                    if (!robot.shootingSequence) {
                        robot.drive.followPath(collectSpike1)  ;
                        robot.intake.setPower(IntakeMotor.States.Collect);
                        timer.reset();
                        state++;
                    }
                    break;
                case 8:
                    if (robot.isDone()) {
                        robot.intake.setPower(IntakeMotor.States.Collect);
                        robot.drive.followPath(preOpenGate, true);
                        state++;
                    }
                    break;
                case 9:
                    if (robot.drive.atParametricEnd()) {
                        robot.drive.followPath(openGate);
                        state++;
                    }
                    break;
                case 10:
                    if (robot.drive.atParametricEnd()) {
                        timer.reset();
                        state++;
                    }
                    break;
                case 11:
                    if (timer.milliseconds() >= 425) {
                        robot.outtake.alignTurret(shootPose2, 2.5);
                        robot.drive.followPath(shootSpike1, true);
                        state++;
                    }
                    break;
                case 12:
                    if (robot.isDone()) {
                        robot.intake.setPower(IntakeMotor.States.Wait);
                        robot.setAction(Robot.Actions.Shoot);
                        state = 13;
                    }
                    break;
                case 13:
                    if (!robot.shootingSequence) {
                        robot.drive.followPath(preCollectGate, true);
                        robot.intake.setPower(IntakeMotor.States.Spit);
                        timer.reset();
                        state++;
                    }
                    break;
                case 14:
                    if (robot.drive.atParametricEnd()) {
                        timer.reset();
                        state++;
                    }
                    break;
                case 15:
                    if (timer.milliseconds() >= 50) {
                        robot.intake.setPower(IntakeMotor.States.Wait);
                        robot.drive.followPath(collectGate, true);
                        timer.reset();
                        state = 16;
                    }
                    break;
                case 16:
                    if (timer.milliseconds() >= 80) {
                        robot.intake.setPower(IntakeMotor.States.Collect);
                        state++;
                    }
                    break;
                case 17:
                    if (robot.isDone() || robot.intake.isPartial) {
                        timer.reset();
                        state++;
                    }
                    break;
                case 18:
                    if (timer.milliseconds() >= 1100) {
                        robot.drive.setMaxPower(1);
                        robot.outtake.setShootingVelocityForPose(shootPose2);
                        robot.drive.followPath(shootGate, true);
                        robot.drive.setMaxPower(1);
                        robot.shouldAlignTurret = true;
                        robot.intake.setPower(IntakeMotor.States.Collect);
                        state++;
                    }
                    break;
                case 19:
                    if (robot.isDone()) {
                        robot.intake.setPower(IntakeMotor.States.Wait);
                        robot.setAction(Robot.Actions.Shoot);
                        state = 21;
                    }
                    break;
                case 21:
                    if (!robot.shootingSequence) {
                        robot.shouldAlignTurret = false;
                        robot.drive.followPath(preCollectLoading2, true);
                        robot.intake.setPower(IntakeMotor.States.Collect);
                        timer.reset();
                        state++;
                    }
                    break;
                case 22:
                    if (robot.drive.atParametricEnd()  || robot.intake.isPartial) {
                        robot.drive.followPath(collectLoading2, true);
                        robot.drive.setMaxPower(0.75);
                        timer.reset();
                        state++;
                    }
                    break;
                case 23:
                    if (robot.drive.atParametricEnd() || robot.intake.isPartial) {
                        timer.reset();
                        state = 24;
                    }
                    break;
                case 24:
                    if ((timer.milliseconds() >= 300  || robot.intake.isFull) && autoTimer.milliseconds() <= 27000) {
                        robot.outtake.alignTurret(shootPose3, -4.5);
                        robot.drive.setMaxPower(1);
                        robot.outtake.setShootingVelocityForPose(shootPose3);
                        robot.outtake.turret.setOffset(0);
                        robot.drive.setMaxPower(1);
                        robot.drive.followPath(shootLoading2Final, true);
                        timer.reset();
                        state++;
                    }
                    break;
                case 25:
                    if (robot.isDone()) {
                        robot.intake.setPower(IntakeMotor.States.Wait);
                        robot.setAction(Robot.Actions.Shoot);
                        timer.reset();
                        state++;
                    }
                    break;
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