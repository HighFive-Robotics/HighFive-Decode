package org.firstinspires.ftc.teamcode.OpModes.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Constants;

@TeleOp(name = "Limelight MT1 Raw Test", group = "Tests")
public class LimelightRawTest extends LinearOpMode {

    Limelight3A limelight;
    Follower drive;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = Constants.createFollower(hardwareMap);
        drive.startFieldCentricDrive(gamepad1, true, 0);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(1);
        limelight.start();

        telemetry.addLine("Wait for Start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            drive.update();
            double robotHeading = Math.toDegrees(drive.getPose().getHeading());
            limelight.updateRobotOrientation(robotHeading);
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                Pose3D botpose = result.getBotpose();

                if (botpose != null) {
                    double xMeters = botpose.getPosition().x;
                    double yMeters = botpose.getPosition().y;
                    double yawDegrees = botpose.getOrientation().getYaw();
                    double xInches = xMeters * 39.37;
                    double yInches = yMeters * 39.37;
                    telemetry.addData("Status", "MT1 Lock Acquired");
                    telemetry.addData("MT1 X (Meters)", "%.3f", xMeters);
                    telemetry.addData("MT1 Y (Meters)", "%.3f", yMeters);
                    telemetry.addData("MT1 Yaw (Degrees)", "%.2f", yawDegrees);
                    telemetry.addData("MT1 X (Inches)", "%.2f", xInches);
                    telemetry.addData("MT1 Y (Inches)", "%.2f", yInches);
                } else {
                    telemetry.addData("Status", "Valid Result, but No Botpose (Check Field Map JSON)");
                }
            } else {
                telemetry.addData("Status", "No Valid Target Found");
            }
            telemetry.addData("Drive Pose", drive.getPose().toString());
            telemetry.update();
            if (gamepad1.psWasPressed()) drive.resetTeleOpHeading();
        }
        limelight.stop();
    }
}