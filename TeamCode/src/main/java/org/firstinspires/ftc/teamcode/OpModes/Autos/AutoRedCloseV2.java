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
import org.firstinspires.ftc.teamcode.Core.Module.Intake.IntakeMotor;
import org.firstinspires.ftc.teamcode.Core.Robot;

@Autonomous(name = "🔴AutoCloseSpikes🔴")
public class AutoRedCloseV2 extends LinearOpMode {

    public Robot robot;
    public int state = 0;

    public Pose startPose = new Pose(125, 114, Math.toRadians(180));//    public Pose startPose = new Pose(13, 113, Math.toRadians(0));

    public Pose shootPose1 = new Pose(93, 95, Math.toRadians(0));
    public Pose shootPose2 = new Pose(83, 81.5, Math.toRadians(0));
    public Pose shootPose3 = new Pose(78, 110, Math.toRadians(0));

    public Pose preCollectSpikeMark2Pose = new Pose(83, 61.5, Math.toRadians(0));
    public Pose collectSpikeMark2Pose = new Pose(127, 61.5, Math.toRadians(0));
    public Pose controlPointSpike2 = new Pose(90, 60);

    public Pose collectSpikeMark1Pose = new Pose(120, 81.5, Math.toRadians(0));
    public Pose preOpenGatePose = new Pose(117, 68.5, Math.toRadians(-90));
    public Pose openGatePose = new Pose(123, 68.5, Math.toRadians(-90));
    public Pose controlPointGate = new Pose(85, 67.5);

//    public Pose collectSpikeMark1Pose = new Pose(18, 81.5, Math.toRadians(180));
//    public Pose controlPointGate = new Pose(53, 67.5);
    public Pose preCollectSpikeMark3Pose = new Pose(93, 36, Math.toRadians(0));
    public Pose collectSpikeMark3Pose = new Pose(125.5, 36, Math.toRadians(0));

    private final ElapsedTime autoTimer = new ElapsedTime();
    private final ElapsedTime timer = new ElapsedTime();

    boolean failsafe = true;

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

        PathChain preOpenGate = robot.drive.pathBuilder()
                .addPath(new BezierLine(collectSpikeMark2Pose, preOpenGatePose))
                .setLinearHeadingInterpolation(collectSpikeMark2Pose.getHeading(), preOpenGatePose.getHeading())
                .build();

        PathChain openGate = robot.drive.pathBuilder()
                .addPath(new BezierLine(preOpenGatePose, openGatePose))
                .setLinearHeadingInterpolation(preOpenGatePose.getHeading(), openGatePose.getHeading())
                .build();

        PathChain shootSpike2 = robot.drive.pathBuilder()
                .addPath(new BezierCurve(openGatePose,controlPointSpike2, shootPose2))
                .setLinearHeadingInterpolation(Math.toRadians(-90),Math.toRadians(0))
                .build();

        PathChain collectSpike1 = robot.drive.pathBuilder()
                .addPath(new BezierLine(shootPose2, collectSpikeMark1Pose))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        PathChain shootSpike1 = robot.drive.pathBuilder()
                .addPath(new BezierCurve(collectSpikeMark1Pose, controlPointGate, shootPose2))
                .setLinearHeadingInterpolation(collectSpikeMark1Pose.getHeading(), shootPose2.getHeading())
                .build();

        PathChain goForSpike3 = robot.drive.pathBuilder()
                .addPath(new BezierLine(shootPose2, preCollectSpikeMark3Pose))
                .setLinearHeadingInterpolation(shootPose2.getHeading(),preCollectSpikeMark3Pose.getHeading())
                .build();

        PathChain collectSpike3 = robot.drive.pathBuilder()
                .addPath(new BezierLine(preCollectSpikeMark3Pose, collectSpikeMark3Pose))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        PathChain shootSpike3Final = robot.drive.pathBuilder()
                .addPath(new BezierLine(collectSpikeMark3Pose, shootPose3))
                .setLinearHeadingInterpolation(Math.toRadians(0),Math.toRadians(0))
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
                        robot.outtake.setShootingVelocityForPose(shootPose1 , -2);
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
                        robot.outtake.alignTurret(shootPose2 , -2);
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
                    if (robot.isDone()) {
                        robot.intake.setPower(IntakeMotor.States.Collect);
                        robot.drive.followPath(preOpenGate, true);
                        state++;
                    }
                    break;
                case 6:
                    if (robot.drive.atParametricEnd()) {
                        robot.drive.followPath(openGate);
                        state++;
                    }
                    break;
                case 7:
                    if (robot.drive.atParametricEnd()) {
                        timer.reset();
                        state++;
                    }
                    break;
                case 8:
                    if (timer.milliseconds() >= 425) {
                        robot.outtake.alignTurret(shootPose2, -2.5);
                        robot.drive.followPath(shootSpike2, true);
                        state++;
                    }
                    break;
                case 9:
                    if (robot.isDone()) {
                        robot.intake.setPower(IntakeMotor.States.Wait);
                        robot.setAction(Robot.Actions.Shoot);
                        robot.setAction(Robot.Actions.ResetTurretCamera);
                        state++;
                    }
                    break;
                case 10:
                    if (!robot.shootingSequence) {
                        robot.drive.followPath(collectSpike1);
                        robot.intake.setPower(IntakeMotor.States.Collect);
                        robot.setAction(Robot.Actions.StopCamera);
                        timer.reset();
                        state++;
                    }
                    break;
                case 11:
                    if (robot.drive.atParametricEnd()) {
                        robot.intake.setPower(IntakeMotor.States.Collect);
                        robot.outtake.alignTurret(shootPose2, 2);
                        robot.drive.followPath(shootSpike1, true);
                        state++;
                    }
                    break;
                case 12:
                    if (robot.isDone()) {
                        robot.intake.setPower(IntakeMotor.States.Wait);
                        robot.setAction(Robot.Actions.Shoot);
                        robot.setAction(Robot.Actions.ResetTurretCamera);
                        state = 13;
                    }
                    break;
                case 13:
                    if (!robot.shootingSequence) {
                        robot.drive.followPath(goForSpike3, true);
                        //robot.outtake.setShootingVelocity(robot.outtake.calculateDistanceToGoal(shootPose3)-2);
                        robot.outtake.setShootingVelocityForPose(shootPose3, -2);
                        robot.outtake.alignTurret(shootPose3, -5);
                        robot.intake.setPower(IntakeMotor.States.Collect);
                        robot.setAction(Robot.Actions.StopCamera);
                        timer.reset();
                        state++;
                    }
                    break;
                case 14:
                    if (robot.drive.atParametricEnd()) {
                        robot.drive.followPath(collectSpike3, true);
                        timer.reset();
                        state++;
                    }
                    break;
                case 15:
                    if (robot.drive.atParametricEnd()) {
                        robot.drive.followPath(shootSpike3Final, true);
                        timer.reset();
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
            }

            if(state == 3){
                robot.outtake.alignTurret(shootPose1);
            }
            if(state == 7){
                robot.outtake.alignTurret(shootPose2);
            }
            if(state == 13){
                robot.outtake.alignTurret(shootPose2);
            }
            if(state == 17){
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