package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;

@TeleOp
public class GeneratedPath extends LinearOpMode {
    public Follower follower;
    public static final Pose startPose = new Pose(26.164, 48.646, 0);
    public BezierCurve pathToFollow =  new BezierCurve(
            new Pose(26.164, 48.646),
            new Pose(67.252, 52.328),
            new Pose(22.288, 125.007),
            new Pose(120.162, 116.092),
            new Pose(57.174, 74.035),
            new Pose(39.925, 24.420),
            new Pose(136.829, 90.509),
            new Pose(97.098, 28.878)
    );

    @Override
    public void runOpMode() throws InterruptedException {
        follower = Constants.createFollower(hardwareMap);
        Path path = new Path(pathToFollow);

        follower.setStartingPose(startPose);
        follower.followPath(path);

        waitForStart();
        while (opModeIsActive()){
            follower.update();
            if(!follower.isBusy()){
                stop();
            }
        }
    }
}
