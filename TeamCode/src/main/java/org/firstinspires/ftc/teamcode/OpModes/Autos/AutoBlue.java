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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Core.Module.Intake.Intake;
import org.firstinspires.ftc.teamcode.Core.Module.Intake.IntakeMotor;
import org.firstinspires.ftc.teamcode.Core.Module.Intake.Sorter;
import org.firstinspires.ftc.teamcode.Core.Module.Outtake.BlockerOuttake;
import org.firstinspires.ftc.teamcode.Core.Robot;

@Disabled
@Autonomous(name = "🔵L🔵")
public class AutoBlue extends LinearOpMode {

    public Robot robot;
    public int state = 0;
    public double velocity1 = 3.3, velocity2 = 3.65;
    boolean spikeMark3 = true;

    public Pose startPose = new Pose(16, 112, Math.toRadians(0));// TODO REMAKE ALL POSES
    public Pose shootPose = new Pose(60, 80, Math.toRadians(-50));
    public Pose openGatePose = new Pose(14, 70);
    public Pose gatePose = new Pose(14, 70, Math.toRadians(135));
    public Pose controlPointGate = new Pose(55, 45);
    public Pose controlPoint1 = new Pose(70, 80);
    public Pose controlPoint2 = new Pose(42, 38);
    public Pose precollectSpikeMark2Pose = new Pose(50, 60, Math.toRadians(180));
    public Pose collectSpikeMark2Pose = new Pose(12, 60, Math.toRadians(180));
    public Pose collectSpikeMark1Pose = new Pose(22, 80, Math.toRadians(180));
    public Pose precollectSpikeMark3Pose = new Pose(52, 35, Math.toRadians(180));
    public Pose collectSpikeMark3Pose = new Pose(12, 35, Math.toRadians(180));

    private final ElapsedTime autoTimer = new ElapsedTime();
    private final ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, startPose, true, Constants.Color.Blue, telemetry, gamepad1);
        autoColor = Constants.Color.Blue;
        robot.drive.resetTeleOpHeading();
        robot.camera.startCapture();
        robot.drive.setConstants(Constants.FConstants);
        robot.intake.setCollectType(Intake.CollectTypes.Normal);
        robot.intake.setState(Intake.States.Wait);

        PathChain preloadPath = robot.drive.pathBuilder()//TODO BEZIER
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        PathChain goForSpike2 = robot.drive.pathBuilder()
                .addPath( new BezierCurve(
                        shootPose,
                        controlPoint1,
                        precollectSpikeMark2Pose
                ))
                .setTangentHeadingInterpolation()
                .build();

        PathChain collectSpike2 = robot.drive.pathBuilder()
                .addPath(new BezierLine(precollectSpikeMark2Pose, collectSpikeMark2Pose))
                .setLinearHeadingInterpolation(precollectSpikeMark2Pose.getHeading(), collectSpikeMark2Pose.getHeading())
                .build();
        PathChain goShoot2 = robot.drive.pathBuilder()
                .addPath(new BezierLine(gatePose, shootPose))
                .setLinearHeadingInterpolation(gatePose.getHeading(), shootPose.getHeading())
                .build();
        PathChain openGate = robot.drive.pathBuilder()//TODO Make another pose
                .addPath( new BezierCurve(
                        collectSpikeMark2Pose,
                        controlPointGate,
                        gatePose
                ))
                .setLinearHeadingInterpolation(collectSpikeMark2Pose.getHeading(),gatePose.getHeading())
                .build();
        PathChain collectSpike1 = robot.drive.pathBuilder()
                .addPath(new BezierLine(shootPose, collectSpikeMark1Pose))
                .setLinearHeadingInterpolation(collectSpikeMark1Pose.getHeading(), collectSpikeMark1Pose.getHeading())
                .build();
        PathChain goForShoot1 = robot.drive.pathBuilder()
                .addPath(new BezierLine(collectSpikeMark1Pose, shootPose))
                .setLinearHeadingInterpolation(collectSpikeMark1Pose.getHeading(), Math.toRadians(180))
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

        PathChain goShootSpike3 = robot.drive.pathBuilder()
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


        robot.intake.sorter.setColor(Constants.Color.Purple,1);
        robot.intake.sorter.setColor(Constants.Color.Purple,2);
        robot.intake.sorter.setColor(Constants.Color.Purple,3);
        telemetry.addLine("Ready for Action");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()){

            finalAutoPose = robot.drive.getPose();
            robot.update();
        }
    }
}
