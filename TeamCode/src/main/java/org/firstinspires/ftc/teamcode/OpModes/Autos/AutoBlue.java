package org.firstinspires.ftc.teamcode.OpModes.Autos;

import static org.firstinspires.ftc.teamcode.Constants.Globals.autoColor;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Core.Robot;

@Autonomous(name = "🔵L🔵")
public class AutoBlue extends LinearOpMode {

    public Robot robot;
    public int state = 0;
    public double velocity = 3.3;

    public Pose startPose = new Pose(16, 112, Math.toRadians(0));
    public Pose shootPreloadPose = new Pose(44, 102.5, Math.toRadians(-42));
    public Pose shootPose = new Pose(60, 80, Math.toRadians(-37));
    public Pose gatePose = new Pose(14, 70, Math.toRadians(20));
    public Pose controlPointGate = new Pose(30, 65);
    public Pose controlPoint1 = new Pose(70, 80);
    public Pose controlPoint2 = new Pose(42, 38);
    public Pose precollectSpikeMark2Pose = new Pose(35, 60, Math.toRadians(180));
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
        PathChain rotateForSpike1 = robot.drive.pathBuilder()
                .addPath(new BezierLine(shootPose, shootPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), Math.toRadians(180))
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
                    robot.shooter.setTargetVelocity(velocity);
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
                    if(robot.isSorterEmpty()){
                        robot.drive.followPath(goForSpike2, true);
                        state++;
                    }
                    break;
            }
            robot.update();
        }
    }
}
