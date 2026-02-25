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
public class AutoBlueGate extends LinearOpMode {

    public Robot robot;
    public int state = 0;

    public Pose startPose = new Pose(13, 113, Math.toRadians(0));

    public Pose shootPose1 = new Pose(45, 95, Math.toRadians(180));
    public Pose shootPose2 = new Pose(55, 83, Math.toRadians(180));
    public Pose shootPose3 = new Pose(50, 120, Math.toRadians(180));

    public Pose preCollectSpikeMark2Pose = new Pose(45, 60, Math.toRadians(180));
    public Pose collectSpikeMark2Pose = new Pose(12, 60, Math.toRadians(180));
    public Pose controlPointSpike2 = new Pose(48.5, 61);

    public Pose collectSpikeMark1Pose = new Pose(18, 83, Math.toRadians(180));
    public Pose preOpenGatePose = new Pose(20, 70, Math.toRadians(180));
    public Pose openGatePose = new Pose(15.5, 70, Math.toRadians(180));
    public Pose controlPointGate = new Pose(53, 67.5);

    public Pose preCollectGatePose = new Pose(19, 68.5, Math.toRadians(-150));
    public Pose controlPointCollectGate1 = new Pose(55, 68.5);
    public Pose controlPointCollectGate2 = new Pose(24, 70.5);
    public Pose collectGatePose = new Pose(7, 55, Math.toRadians(110));
    public Pose controlPointGateCollectGate1 = new Pose(16, 47);
    public Pose controlPointGateCollectGate2 = new Pose(50, 69);
    public Pose controlPointGateCollectFinalGate1 = new Pose(19, 40.5);
    public Pose controlPointGateCollectFinalGate2 = new Pose(37, 99.5);

    public Pose preCollectSpikeMark3Pose = new Pose(44, 34, Math.toRadians(180));
    public Pose collectSpikeMark3Pose = new Pose(16, 34, Math.toRadians(180));
    public Pose collectLoadingZone1 = new Pose(10, 12, Math.toRadians(180));
    public Pose preCollectLoadingZone1 = new Pose(10, 22, Math.toRadians(180));

    private final ElapsedTime autoTimer = new ElapsedTime();
    private final ElapsedTime timer = new ElapsedTime();

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
        robot.shouldAlignTurret = false;//TODO
        telemetry.setMsTransmissionInterval(500);
        PathChain preloadPath = robot.drive.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose1))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose1.getHeading())
                .build();

        PathChain goForSpike2 = robot.drive.pathBuilder()
                .addPath(new BezierLine(shootPose1, preCollectSpikeMark2Pose))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        PathChain collectSpike2 = robot.drive.pathBuilder()
                .addPath(new BezierLine(preCollectSpikeMark2Pose, collectSpikeMark2Pose))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        PathChain shootSpike2 = robot.drive.pathBuilder()
                .addPath(new BezierCurve(collectSpikeMark2Pose,controlPointSpike2, shootPose2))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        PathChain collectSpike1 = robot.drive.pathBuilder()
                .addPath(new BezierLine(shootPose2, collectSpikeMark1Pose))
                .setConstantHeadingInterpolation(Math.toRadians(180))
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
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        PathChain preCollectGate = robot.drive.pathBuilder()
                .addPath(new BezierCurve(shootPose2,controlPointCollectGate1,controlPointCollectGate2, preCollectGatePose))
                .setTangentHeadingInterpolation()
                .build();

        PathChain collectGate = robot.drive.pathBuilder()
                .addPath(new BezierLine(preCollectGatePose, collectGatePose))
                .setLinearHeadingInterpolation(preCollectGatePose.getHeading(),collectGatePose.getHeading())
                .build();

        PathChain shootGate = robot.drive.pathBuilder()
                .addPath(new BezierCurve(collectGatePose,controlPointGateCollectGate1,controlPointGateCollectGate2, shootPose2))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        PathChain shootGateFinal = robot.drive.pathBuilder()
                .addPath(new BezierCurve(collectGatePose,controlPointGateCollectFinalGate1,controlPointGateCollectFinalGate2, shootPose3))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();
                       /*

PathChain ca = robot.drive.pathBuilder()
                .addPath(new BezierLine(shootPose, collectSpikeMark1Pose))
                .setLinearHeadingInterpolation(collectSpikeMark1Pose.getHeading(), collectSpikeMark1Pose.getHeading())
                .build();

        PathChain goForShoot1 = robot.drive.pathBuilder()
                .addPath(new BezierLine(collectSpikeMark1Pose, shootPose))
                .setLinearHeadingInterpolation(collectSpikeMark1Pose.getHeading(), Math.toRadians(180))
                .build();

        PathChain goForSpike3 = robot.drive.pathBuilder()
                .addPath(new BezierCurve(shootPose, controlPoint3, preCollectSpikeMark3Pose))
                .setTangentHeadingInterpolation()
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
                .addPath(new BezierCurve(shootPose, controlPoint4, collectLoadingZone1))
                .setTangentHeadingInterpolation()
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
                .addPath(new BezierCurve(collectLoadingZone1, controlPoint4, lastShootPose))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();*/

        telemetry.addLine("Ready for Action");
        telemetry.update();
        robot.update();

        waitForStart();
        autoTimer.reset();
        //robot.outtake.setShootingVelocity(120);
        while (opModeIsActive()) {
            switch (state) {
                case 0:
                    robot.drive.followPath(preloadPath);
                    state++;
                    break;
                case 1:
                    if(robot.isDone()){
                        robot.drive.followPath(goForSpike2);
                        state++;
                    }
                    break;
                case 2:
                    if(robot.isDone()){
                        robot.drive.followPath(collectSpike2);
                        state++;
                    }
                    break;
                case 3:
                    if(robot.isDone()){
                        robot.drive.followPath(shootSpike2);
                        state++;
                    }
                    break;
                case 4:
                    if(robot.isDone()){
                        robot.drive.followPath(collectSpike1);
                        state++;
                    }
                    break;
                case 5:
                    if(robot.isDone()){
                        robot.drive.followPath(preOpenGate);
                        state++;
                    }
                    break;
                case 6:
                    if(robot.isDone()){
                        robot.drive.setMaxPower(0.7);
                        robot.drive.followPath(openGate);
                        state++;
                    }
                    break;
                case 7:
                    if(robot.isDone()){
                        robot.drive.setMaxPower(1);
                        robot.drive.followPath(shootSpike1);
                        state++;
                    }
                    break;
                case 8:
                    if(robot.isDone()){
                        robot.drive.followPath(preCollectGate);
                        state++;
                    }
                    break;
                case 9:
                    if(robot.isDone()){
                        robot.drive.followPath(collectGate);
                        robot.intake.setPower(IntakeMotor.States.Collect);
                        state++;
                    }
                    break;
                case 10:
                    if(robot.isDone()){
                        robot.drive.followPath(shootGate);
                        robot.intake.setPower(IntakeMotor.States.Wait);
                        state++;
                    }
                    break;
                case 11:
                    if(robot.isDone()){
                        robot.drive.followPath(preCollectGate);
                        state++;
                    }
                    break;
                case 12:
                    if(robot.isDone()){
                        robot.drive.followPath(collectGate);
                        robot.intake.setPower(IntakeMotor.States.Collect);
                        state++;
                    }
                    break;
                case 13:
                    if(robot.isDone()){
                        robot.drive.followPath(shootGateFinal);
                        robot.intake.setPower(IntakeMotor.States.Wait);
                        state++;
                    }
                    break;
            }

            finalAutoPose = robot.drive.getPose();
            robot.update();
            telemetry.addData("State: ", state);
            telemetry.addData("Distance: ", robot.outtake.distanceToGoal);
            telemetry.addData("Target Up: ", robot.outtake.shooter.getTargetUp());
            telemetry.addData("Velo Up: ", robot.outtake.shooter.getVelocityErrorUp());
            telemetry.addData("Target Down: ", robot.outtake.shooter.getTargetDown());
            telemetry.addData("Velo Down: ", robot.outtake.shooter.getVelocityErrorDown());
            telemetry.addData("State: ", state);

            // telemetry.addData("Distance:", robot.outtake.distanceToGoal);
            telemetry.update();
        }
        robot.outtake.stopBreakBeamThread();
        Constants.CameraConstants.toleranceTurretDeg = 1;
    }
}