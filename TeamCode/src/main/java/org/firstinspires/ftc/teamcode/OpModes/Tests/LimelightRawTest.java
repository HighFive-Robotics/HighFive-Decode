package org.firstinspires.ftc.teamcode.OpModes.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Core.Hardware.HighCamera;
import org.firstinspires.ftc.teamcode.Core.Module.Outtake.LinkageCamera;
import org.firstinspires.ftc.teamcode.Core.Module.Outtake.Outtake;

@TeleOp(name = "Limelight MT1 Raw Test", group = "Tests")
public class LimelightRawTest extends LinearOpMode {

    Follower drive;
    HighCamera camera;
    Outtake outtake;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = Constants.createFollower(hardwareMap);
        drive.setStartingPose(new Pose(6,6,0));
        drive.setPose(new Pose(6,6,0));
        drive.startFieldCentricDrive(gamepad1, true, 0);
        camera = new HighCamera(hardwareMap , HighCamera.Pipelines.BallDetection);
        outtake = new Outtake(hardwareMap, Constants.Color.Red , telemetry , true);
        outtake.linkageCamera.setState(LinkageCamera.States.Artifact , 300);
        Pose cameraPose = new Pose(6,6,0);
        Path path = new Path(new BezierLine(new Pose(6,6,0) , new Pose(6,6,0)));
        String data = "NO DATA YET";
        telemetry.addLine("Wait for Start");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
            if(gamepad1.squareWasPressed()){
                outtake.linkageCamera.setState(LinkageCamera.States.Artifact , 300);
            }
            if(gamepad1.circleWasPressed()){
                outtake.linkageCamera.setState(LinkageCamera.States.Goal , 300);
            }
            if(gamepad1.crossWasPressed()){
                data = camera.getBallInfo();
                cameraPose = camera.getBallPose(drive.getPose());
                path = new Path(
                        new BezierLine(
                                drive.getPose(),
                                cameraPose
                        )
                );
            }
            if(gamepad1.triangleWasPressed()){
                drive.followPath(path);
            }
            if (gamepad1.psWasPressed()) drive.resetTeleOpHeading();

            telemetry.addData("BALL INFO" , data);
            telemetry.update();
            drive.update();
            outtake.update(drive.getPose());
        }

    }
}