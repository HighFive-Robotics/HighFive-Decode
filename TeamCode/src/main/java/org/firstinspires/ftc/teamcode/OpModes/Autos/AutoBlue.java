package org.firstinspires.ftc.teamcode.OpModes.Autos;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Core.Robot;

@Autonomous
public class AutoBlue extends LinearOpMode {
    public Robot robot;
    private int state = 1;

    public Pose startPose = new Pose(16, 112, Math.toRadians(0));
    private final Pose shootPose = new Pose(65, 95, Math.toRadians(-50));

    private final Pose spike1Pose = new Pose(55, 95, Math.toRadians(180));
    private final Pose collect1Pose = new Pose(16, 95, Math.toRadians(180));
    private final Pose spike2Pose = new Pose(55, 70, Math.toRadians(180));
    private final Pose collect2Pose = new Pose(10, 70, Math.toRadians(180));
    private final Pose spike3Pose = new Pose(55, 45, Math.toRadians(180));
    private final Pose collect3Pose = new Pose(10, 45, Math.toRadians(180));

    private ElapsedTime timer = new ElapsedTime();

    private final PathChain preloadPath = robot.drive.pathBuilder()
            .addPath(new BezierLine(startPose , shootPose))
            .setLinearHeadingInterpolation(startPose.getHeading(),shootPose.getHeading())
            .build();
    private final PathChain goForSpike1 = robot.drive.pathBuilder()
            .addPath(new BezierLine(shootPose , spike1Pose))
            .setLinearHeadingInterpolation(shootPose.getHeading(),spike1Pose.getHeading())
            .build();
    private final PathChain collectSpike1 = robot.drive.pathBuilder()
            .addPath(new BezierLine(spike1Pose , collect1Pose))
            .setLinearHeadingInterpolation(spike1Pose.getHeading(),collect1Pose.getHeading())
            .build();
    private final PathChain shootFromSpike1 =robot.drive.pathBuilder()
            .addPath(new BezierLine(collect1Pose , shootPose))
            .setLinearHeadingInterpolation(collect1Pose.getHeading(),shootPose.getHeading())
            .build();
    private final PathChain goForSpike2 =robot.drive.pathBuilder()
            .addPath(new BezierLine(shootPose , spike2Pose))
            .setLinearHeadingInterpolation(shootPose.getHeading(),spike2Pose.getHeading())
            .build();
    private final PathChain collectSpike2 =robot.drive.pathBuilder()
            .addPath(new BezierLine(spike2Pose , collect2Pose))
            .setLinearHeadingInterpolation(spike2Pose.getHeading(),collect2Pose.getHeading())
            .build();
    private final PathChain shootFromSpike2 = robot.drive.pathBuilder()
            .addPath(new BezierLine(collect2Pose , shootPose))
            .setLinearHeadingInterpolation(collect2Pose.getHeading(),shootPose.getHeading())
            .build();
    private final PathChain goForSpike3 =robot.drive.pathBuilder()
            .addPath(new BezierLine(shootPose , spike3Pose))
            .setLinearHeadingInterpolation(shootPose.getHeading(),spike3Pose.getHeading())
            .build();
    private final PathChain collectSpike3 =robot.drive.pathBuilder()
            .addPath(new BezierLine(spike3Pose , collect3Pose))
            .setLinearHeadingInterpolation(spike3Pose.getHeading(),collect3Pose.getHeading())
            .build();
    private final PathChain shootFromSpike3 =robot.drive.pathBuilder()
            .addPath(new BezierLine(collect3Pose , shootPose))
            .setLinearHeadingInterpolation(collect3Pose.getHeading(),shootPose.getHeading())
            .build();
    private final PathChain goToPark =robot.drive.pathBuilder()
            .addPath(new BezierLine(shootPose , collect1Pose))
            .setLinearHeadingInterpolation(shootPose.getHeading(),collect1Pose.getHeading())
            .build();


    @Override
    public void runOpMode() throws InterruptedException {

    }
}
