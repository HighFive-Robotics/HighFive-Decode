package org.firstinspires.ftc.teamcode.OpModes.Autos;

import static org.firstinspires.ftc.teamcode.Constants.Globals.autoColor;
import static org.firstinspires.ftc.teamcode.Constants.Globals.finalAutoPose;
import static org.firstinspires.ftc.teamcode.Constants.Globals.randomizedCase;
import static org.firstinspires.ftc.teamcode.Constants.Intake.SorterConstants.sorterColors;
import static org.firstinspires.ftc.teamcode.Core.Robot.Actions.PrepareForShooting;
import static org.firstinspires.ftc.teamcode.Core.Robot.Actions.ShootFastNormal;

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
import org.firstinspires.ftc.teamcode.Core.Module.Intake.Sorter;
import org.firstinspires.ftc.teamcode.Core.Module.Outtake.BlockerOuttake;
import org.firstinspires.ftc.teamcode.Core.Robot;

@Autonomous(name = "🔵L🔵")
public class AutoBlue extends LinearOpMode {

    public Robot robot;
    public int state = 0;
    public double velocity1 = 3.3, velocity2 = 3.65;

    public Pose startPose = new Pose(16, 112, Math.toRadians(0));
    public Pose shootPreloadPose = new Pose(44, 102.5, Math.toRadians(-42));
    public Pose shootPose = new Pose(60, 80, Math.toRadians(-50));
    public Pose gatePose = new Pose(14, 70, Math.toRadians(65));
    public Pose controlPointGate = new Pose(55, 45);
    public Pose controlPoint1 = new Pose(70, 80);
    public Pose controlPoint2 = new Pose(42, 38);
    public Pose precollectSpikeMark2Pose = new Pose(50, 60, Math.toRadians(180));
    public Pose collectSpikeMark2Pose = new Pose(12, 60, Math.toRadians(180));
    public Pose precollectSpikeMark1Pose = new Pose(52, 80, Math.toRadians(180));
    public Pose collectSpikeMark1Pose = new Pose(22, 80, Math.toRadians(180));
    public Pose precollectSpikeMark3Pose = new Pose(52, 35, Math.toRadians(180));
    public Pose collectSpikeMark3Pose = new Pose(12, 35, Math.toRadians(180));
    public Pose park = new Pose(15, 83.5, Math.toRadians(180));

    private Robot.Actions shootAction;

    private final ElapsedTime autoTimer = new ElapsedTime();
    private final ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, startPose, true, Constants.Color.Blue, telemetry, gamepad1);
        autoColor = Constants.Color.Blue;
        robot.drive.resetTeleOpHeading();
        robot.camera.startCapture();
        robot.drive.setConstants(Constants.FConstants);

        PathChain preloadPath = robot.drive.pathBuilder()
                .addPath(new BezierLine(startPose, shootPreloadPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPreloadPose.getHeading())
                .build();

        PathChain goForSpike2 = robot.drive.pathBuilder()
                .addPath( new BezierCurve(
                        shootPreloadPose,
                        controlPoint1,
                        precollectSpikeMark2Pose
                ))
                .setTangentHeadingInterpolation()
                .build();

        PathChain collectSpike2 = robot.drive.pathBuilder()
                .addPath(new BezierLine(precollectSpikeMark2Pose, collectSpikeMark2Pose))
                .setLinearHeadingInterpolation(precollectSpikeMark2Pose.getHeading(), collectSpikeMark2Pose.getHeading())
                .build();

        PathChain openGate = robot.drive.pathBuilder()
                .addPath( new BezierCurve(
                        collectSpikeMark2Pose,
                        controlPointGate,
                        gatePose
                ))
                .setLinearHeadingInterpolation(collectSpikeMark2Pose.getHeading(),gatePose.getHeading())
                .build();

        PathChain goShoot2 = robot.drive.pathBuilder()
                .addPath(new BezierLine(gatePose, shootPose))
                .setLinearHeadingInterpolation(gatePose.getHeading(), shootPose.getHeading())
                .build();
        PathChain goForSpike1 = robot.drive.pathBuilder()
                .addPath(new BezierLine(shootPose, precollectSpikeMark1Pose))
                .setLinearHeadingInterpolation(Math.toRadians(180), precollectSpikeMark1Pose.getHeading())
                .build();
        PathChain collectSpike1 = robot.drive.pathBuilder()
                .addPath(new BezierLine(precollectSpikeMark1Pose, collectSpikeMark1Pose))
                .setLinearHeadingInterpolation(precollectSpikeMark1Pose.getHeading(), collectSpikeMark1Pose.getHeading())
                .build();
        PathChain goForShoot1 = robot.drive.pathBuilder()
                .addPath(new BezierLine(collectSpikeMark1Pose, shootPose))
                .setLinearHeadingInterpolation(collectSpikeMark1Pose.getHeading(), Math.toRadians(180))
                .build();
        PathChain rotateShoot1 = robot.drive.pathBuilder()
                .addPath(new BezierLine(shootPose, shootPose))
                .setLinearHeadingInterpolation(Math.toRadians(180), shootPose.getHeading())
                .build();

        PathChain goForSpike3 = robot.drive.pathBuilder()
                .addPath( new BezierCurve(
                        shootPose,
                        controlPoint2,
                        precollectSpikeMark3Pose
                ))
                .setTangentHeadingInterpolation()
                .build();

        PathChain collectSpike3 = robot.drive.pathBuilder()
                .addPath(new BezierLine(precollectSpikeMark3Pose, collectSpikeMark3Pose))
                .setLinearHeadingInterpolation(precollectSpikeMark3Pose.getHeading(), collectSpikeMark3Pose.getHeading())
                .build();

        PathChain goShootZbrrrrVrumVrum = robot.drive.pathBuilder()
                .addPath( new BezierCurve(
                        collectSpikeMark3Pose,
                        controlPoint2,
                        shootPose
                ))
                .setTangentHeadingInterpolation()
                .build();

        PathChain shoot3 = robot.drive.pathBuilder()
                .addPath(new BezierLine(shootPose, shootPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), shootPose.getHeading())
                .build();

        PathChain parkPath = robot.drive.pathBuilder()
                .addPath(new BezierLine(shootPose, park))
                .setLinearHeadingInterpolation(shootPose.getHeading(), park.getHeading())
                .build();


        robot.intake.sorter.setColor(Constants.Color.Purple,1);
        robot.intake.sorter.setColor(Constants.Color.Purple,2);
        robot.intake.sorter.setColor(Constants.Color.Purple,3);
        telemetry.addLine("Ready for Action");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()){
            switch (state){
                case 0:
                    robot.drive.followPath(preloadPath, true);
                    robot.shooter.setTargetVelocity(velocity1);
                    state++;
                    break;
                case 1:
                    if((robot.isDone() && robot.shooter.atTarget()) || autoTimer.milliseconds() > 5000){
                        robot.setAction(Robot.Actions.ShootFast);
                        timer.reset();
                        state++;
                    }
                    break;
                case 2:
                    if(robot.isSorterEmpty() || timer.milliseconds() >= 3000){
                        robot.intake.setCollectType(Intake.CollectTypes.Normal);
                        robot.drive.followPath(goForSpike2, true);
                        state++;
                    }
                    break;
                case 3://TODO Might not work
                    if(robot.isDone()){
                        robot.drive.followPath(collectSpike2, true);
                        robot.intake.setPower(IntakeMotor.States.Collect);
                        robot.shooter.setTargetVelocity(-1);
                        state++;
                        timer.reset();
                    }
                    break;
                case 4:
                    if(robot.isDone() || robot.intake.sorter.isFull){
                        robot.drive.followPath(openGate, true);
                        state++;
                    }
                    break;
                case 5:
                    if(robot.isDone()){
                        timer.reset();
                        state++;
                    }
                    break;
                case 6:
                    if(timer.milliseconds() >= 1000){
                        robot.drive.followPath(openGate, true);
                        state++;
                    }
                    break;
                case 7:
                    if(robot.isDone()){
                        robot.drive.followPath(goShoot2, true);
                        robot.shooter.setTargetVelocity(velocity2);
                        robot.shooter.blocker.setState(BlockerOuttake.States.Open);
                        state++;
                    }
                    break;
                case 8:
                    if(robot.isDone()){
                        robot.setAction(ShootFastNormal);
                        timer.reset();
                        state = 9;
                    }
                    break;
                case 9:
                    if(!robot.shootNormal){
                        robot.drive.turnToDegrees(180);
                        state++;
                    }
                    break;
                case 10:
                    if(robot.isDone()){
                        robot.drive.followPath(goForSpike1, true);
                        state++;
                    }
                    break;
                case 11:
                    if(robot.isDone()){
                        robot.shooter.setTargetVelocity(-1);
                        robot.intake.setPower(IntakeMotor.States.Collect);
                        robot.drive.followPath(collectSpike1, true);
                        timer.reset();
                        state++;
                    }
                    break;
                case 12:
                    if(robot.isDone() || robot.intake.sorter.isFull){
                        robot.drive.followPath(goForShoot1, true);
                        state++;
                    }
                    break;
                case 13:
                    if(robot.isDone()){
                        robot.drive.turnTo(shootPose.getHeading());
                        state++;
                    }
                    break;
                case 14:
                    if(robot.isDone()){
                        robot.setAction(PrepareForShooting);
                        timer.reset();
                        state = 145;
                    }
                    break;
                case 145:
                    if(timer.milliseconds() >= 500){
                        robot.setAction(shootAction);
                        timer.reset();
                        state = 15;
                    }
                    break;
                case 15:
                    if(!robot.shootNormal){
                        robot.shooter.setTargetVelocity(0);
                        robot.drive.followPath(goForSpike3, true);
                        state++;
                    }
                    break;
                case 16://TODO Might not work
                    if(robot.isDone()){
                        robot.drive.followPath(collectSpike3, true);
                        robot.drive.setMaxPower(0.6);
                        robot.intake.setPower(IntakeMotor.States.Collect);
                        state++;
                    }
                    break;
                case 17:
                    if(robot.isDone() || robot.intake.sorter.isFull){
                        robot.drive.setMaxPower(1);
                        robot.intake.setPower(IntakeMotor.States.Wait);
                        robot.drive.followPath(goShootZbrrrrVrumVrum, true);
                        robot.drive.setMaxPower(1);
                        state++;
                    }
                    break;
                case 18:
                    if(robot.isDone()){
                        robot.drive.followPath(shoot3, true);
                        state++;
                    }
                    break;
                case 19:
                    if(robot.isDone()){
                        robot.setAction(shootAction);
                        timer.reset();
                        state = 20;
                    }
                    break;
                case 20:
                    if(robot.isSorterEmpty() || timer.milliseconds() >= 3000){
                        robot.shooter.setTargetVelocity(0);
                        robot.drive.followPath(parkPath);
                        robot.drive.setMaxPower(1);
                        state = 100;
                    }
                    break;
                case 21:
                    robot.drive.followPath(parkPath);
                    robot.drive.setMaxPower(1);
                    state = 100;
                    break;
            }
            if(autoTimer.milliseconds() >= 28000 && state >= 18){
                robot.shooter.setTargetVelocity(0);
                robot.intake.setPower(IntakeMotor.States.Spit);
                state = 21;
            }
            telemetry.addData("State:", state);
            telemetry.addData("Randomized Case:", randomizedCase);
            telemetry.addData("Color 1:", sorterColors[0]);
            telemetry.addData("Color 2:", sorterColors[1]);
            telemetry.addData("Color 3:", sorterColors[2]);
            telemetry.update();
            finalAutoPose = robot.drive.getPose();
            robot.update();
        }
    }
}
