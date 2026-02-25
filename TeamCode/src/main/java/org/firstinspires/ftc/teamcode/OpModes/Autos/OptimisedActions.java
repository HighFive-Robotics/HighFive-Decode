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

@Autonomous(name = "🔵AutoCloseGate🔵")
public class OptimisedActions extends LinearOpMode {

    public Robot robot;
    public int state = 0;
    public int extraCyclesCompleted = 0;
    public final int extraCycles = 2;

    public Pose startPose = new Pose(13, 113, Math.toRadians(0));
    public Pose parkPose = new Pose(12, 12, Math.toRadians(180));

    public Pose shootPose = new Pose(45, 95, Math.toRadians(-40));
    public Pose lastShootPose = new Pose(50, 112.5, Math.toRadians(-25));

    public Pose preOpenGatePose = new Pose(20, 68, Math.toRadians(-90));
    public Pose openGatePose = new Pose(14, 68, Math.toRadians(-90));

    public Pose controlPoint1 = new Pose(60, 60);
    public Pose controlPoint2 = new Pose(45, 65);
    public Pose controlPoint3 = new Pose(62, 45);
    public Pose controlPoint4 =  new Pose(49, 53);

    public Pose preCollectSpikeMark1Pose = new Pose(44, 82, Math.toRadians(180));
    public Pose collectSpikeMark1Pose = new Pose(20, 82, Math.toRadians(180));

    public Pose preCollectSpikeMark2Pose = new Pose(44, 60, Math.toRadians(180));
    public Pose collectSpikeMark2Pose = new Pose(14.2, 60, Math.toRadians(180));

    public Pose preCollectSpikeMark3Pose = new Pose(44, 34, Math.toRadians(180));
    public Pose collectSpikeMark3Pose = new Pose(16, 34, Math.toRadians(180));

    public Pose preCollectLoadingZone1 = new Pose(10, 22, Math.toRadians(180));
    public Pose collectLoadingZone1 = new Pose(10, 12, Math.toRadians(180));

    private final ElapsedTime autoTimer = new ElapsedTime();
    private final ElapsedTime actionTimer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new Robot(hardwareMap, startPose, true, Constants.Color.Blue, telemetry, gamepad1);
        robot.outtake.turret.reset();
        robot.outtake.startBreakBeamThread();
        autoColor = Constants.Color.Blue;
        robot.drive.resetTeleOpHeading();
        robot.camera.startCapture();
        robot.drive.setConstants(Constants.FConstants);
        Constants.Globals.afterAuto = true;

        robot.shouldAlignTurret = false;
        telemetry.setMsTransmissionInterval(50);

        PathChain preloadPath = robot.drive.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        PathChain collectSpike2 = robot.drive.pathBuilder()
                .addPath(new BezierCurve(shootPose, controlPoint2, preCollectSpikeMark2Pose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), preCollectSpikeMark2Pose.getHeading())
                .addPath(new BezierLine(preCollectSpikeMark2Pose, collectSpikeMark2Pose))
                .setConstantHeadingInterpolation(collectSpikeMark2Pose.getHeading())
                .build();

        PathChain shootSpike2Path = robot.drive.pathBuilder()
                .addPath(new BezierCurve(collectSpikeMark2Pose, controlPoint2, shootPose))
                .setLinearHeadingInterpolation(collectSpikeMark2Pose.getHeading(), shootPose.getHeading())
                .build();

        PathChain collectSpike1 = robot.drive.pathBuilder()
                .addPath(new BezierLine(shootPose, preCollectSpikeMark1Pose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), preCollectSpikeMark1Pose.getHeading())
                .addPath(new BezierLine(preCollectSpikeMark1Pose, collectSpikeMark1Pose))
                .setConstantHeadingInterpolation(collectSpikeMark1Pose.getHeading())
                .build();

        PathChain shootSpike1Path = robot.drive.pathBuilder()
                .addPath(new BezierLine(collectSpikeMark1Pose, shootPose))
                .setConstantHeadingInterpolation(shootPose.getHeading())
                .build();

        PathChain openGatePath = robot.drive.pathBuilder()
                .addPath(new BezierLine(shootPose, preOpenGatePose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), preOpenGatePose.getHeading())
                .addPath(new BezierLine(preOpenGatePose, openGatePose))
                .setConstantHeadingInterpolation(openGatePose.getHeading())
                .build();

        PathChain collectSpike3 = robot.drive.pathBuilder()
                .addPath(new BezierCurve(openGatePose, controlPoint4, preCollectSpikeMark3Pose))
                .setLinearHeadingInterpolation(openGatePose.getHeading(), preCollectSpikeMark3Pose.getHeading())
                .addPath(new BezierLine(preCollectSpikeMark3Pose, collectSpikeMark3Pose))
                .setConstantHeadingInterpolation(collectSpikeMark3Pose.getHeading())
                .build();

        PathChain shootSpike3Path = robot.drive.pathBuilder()
                .addPath(new BezierCurve(collectSpikeMark3Pose, controlPoint3, shootPose))
                .setLinearHeadingInterpolation(collectSpikeMark3Pose.getHeading(), shootPose.getHeading())
                .build();

        PathChain collectExtra = robot.drive.pathBuilder()
                .addPath(new BezierCurve(shootPose, controlPoint4, preCollectLoadingZone1))
                .setLinearHeadingInterpolation(shootPose.getHeading(), preCollectLoadingZone1.getHeading())
                .addPath(new BezierLine(preCollectLoadingZone1, collectLoadingZone1))
                .setConstantHeadingInterpolation(collectLoadingZone1.getHeading())
                .build();

        PathChain shootExtraPath = robot.drive.pathBuilder()
                .addPath(new BezierCurve(collectLoadingZone1, controlPoint3, lastShootPose))
                .setLinearHeadingInterpolation(collectLoadingZone1.getHeading(), lastShootPose.getHeading())
                .build();

        telemetry.addLine("Ready for Action");
        telemetry.update();
        robot.update();

        waitForStart();
        autoTimer.reset();
        robot.drive.setMaxPower(1.0);
        robot.outtake.setShootingVelocityForPose(shootPose);
        while (opModeIsActive()) {
            if (autoTimer.seconds() > 27.5 && state < 90) {
                state = 99;
            }
            switch (state) {
                case 0:
                    robot.drive.followPath(preloadPath);
                    robot.intake.setPower(IntakeMotor.States.Collect);
                    robot.outtake.alignTurret(shootPose);
                    state= 909;
                    break;
                case 909:
                    if(robot.isDone()){
                        robot.setAction(Robot.Actions.ResetTurretCamera);
                        state = 1;
                    }
                case 1:
                    if (robot.isDone() && !robot.resetWithCamera) {
                        robot.setAction(Robot.Actions.Shoot);
                        state++;
                    }
                    break;

                case 2:
                    if (!robot.shootingSequence) {
                        robot.drive.followPath(collectSpike2);
                        robot.intake.setPower(IntakeMotor.States.Collect);
                        actionTimer.reset();
                        state++;
                    }
                    break;
                case 3:
                    if (robot.isDone() && (robot.intake.isFull || actionTimer.milliseconds() > 2500)) {
                        robot.intake.setPower(IntakeMotor.States.Collect);
                        robot.drive.followPath(shootSpike2Path);
                        robot.outtake.setShootingVelocityForPose(shootPose);
                        state++;
                    }
                    break;
                case 4:
                    if (robot.isDone()) {
                        robot.intake.setPower(IntakeMotor.States.Wait);
                        robot.setAction(Robot.Actions.Shoot);
                        state++;
                    }
                    break;

                case 5:
                    if (!robot.shootingSequence) {
                        robot.drive.followPath(collectSpike1);
                        robot.intake.setPower(IntakeMotor.States.Collect);
                        actionTimer.reset();
                        state++;
                    }
                    break;
                case 6:
                    if (robot.isDone() && (robot.intake.isFull || actionTimer.milliseconds() > 2500)) {
                        robot.intake.setPower(IntakeMotor.States.Collect);
                        robot.drive.followPath(shootSpike1Path);
                        robot.outtake.setShootingVelocityForPose(shootPose,2);
                        state++;
                    }
                    break;
                case 7:
                    if (robot.isDone()) {
                        robot.intake.setPower(IntakeMotor.States.Wait);
                        robot.setAction(Robot.Actions.Shoot);
                        state++;
                    }
                    break;

                case 8:
                    if (!robot.shootingSequence) {
                        robot.drive.followPath(openGatePath);
                        state++;
                        actionTimer.reset();
                    }
                    break;
                case 9:
                    if (robot.isDone() && actionTimer.milliseconds() >= 200) {
                        robot.drive.followPath(collectSpike3);
                        robot.intake.setPower(IntakeMotor.States.Collect);
                        robot.outtake.alignTurret(shootPose);
                        actionTimer.reset();
                        state++;
                    }
                    break;
                case 10:
                    if (robot.isDone() && (robot.intake.isFull || actionTimer.milliseconds() > 2500)) {
                        robot.intake.setPower(IntakeMotor.States.Collect);
                        robot.drive.followPath(shootSpike3Path);
                        robot.outtake.setShootingVelocityForPose(shootPose);
                        state++;
                    }
                    break;
                case 11:
                    if (robot.isDone()) {
                        robot.intake.setPower(IntakeMotor.States.Wait);
                        robot.setAction(Robot.Actions.Shoot);
                        state++;
                    }
                    break;

                case 12:
                    if (!robot.shootingSequence) {
                        if (extraCyclesCompleted < extraCycles) {
                            robot.drive.followPath(collectExtra);
                            robot.intake.setPower(IntakeMotor.States.Collect);
                            actionTimer.reset();
                            state++;
                        } else {
                            state = 99;
                        }
                    }
                    break;
                case 13:
                    if (robot.isDone() && (robot.intake.isFull || actionTimer.milliseconds() > 2500)) {
                        robot.intake.setPower(IntakeMotor.States.Collect);
                        robot.outtake.alignTurret(lastShootPose);
                        robot.drive.followPath(shootExtraPath);
                        robot.outtake.setShootingVelocityForPose(lastShootPose);
                        state++;
                    }
                    break;
                case 14:
                    if (robot.isDone()) {
                        robot.intake.setPower(IntakeMotor.States.Wait);
                        robot.setAction(Robot.Actions.Shoot);
                        state++;
                    }
                    break;
                case 15:
                    if (!robot.shootingSequence) {
                        extraCyclesCompleted++;
                        state = 12;
                    }
                    break;

                case 99:
                    robot.setAction(Robot.Actions.StopShoot);
                    robot.intake.setPower(IntakeMotor.States.Wait);

                    PathChain parkChain = robot.drive.pathBuilder()
                            .addPath(new BezierLine(robot.drive.getPose(), parkPose))
                            .setConstantHeadingInterpolation(parkPose.getHeading())
                            .build();

                    robot.drive.followPath(parkChain);
                    state = 100;
                    break;
                case 100:
                    if (robot.isDone()) {
                        state = 101;
                    }
                    break;
            }

            finalAutoPose = robot.drive.getPose();
            robot.update();

            telemetry.addData("State", state);
            telemetry.addData("Timer", autoTimer.seconds());
            telemetry.addData("Extra Cycles Done", extraCyclesCompleted);
            telemetry.addData("Intake Full", robot.intake.isFull);
            telemetry.addData("Shooting Seq Active", robot.shootingSequence);
            telemetry.update();
        }

        robot.outtake.stopBreakBeamThread();
        Constants.CameraConstants.toleranceTurretDeg = 1;
    }
}