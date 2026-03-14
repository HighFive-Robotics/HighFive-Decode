package org.firstinspires.ftc.teamcode.OpModes.Tests;

import static org.firstinspires.ftc.teamcode.Constants.Globals.BlueGoalDistance;
import static org.firstinspires.ftc.teamcode.Constants.Globals.RedGoalDistance;
import static org.firstinspires.ftc.teamcode.Constants.Globals.autoColor;
import static org.firstinspires.ftc.teamcode.Constants.Globals.finalAutoPose;
import static org.firstinspires.ftc.teamcode.Core.Module.Intake.IntakeMotor.States.Collect;
import static org.firstinspires.ftc.teamcode.Core.Module.Intake.IntakeMotor.States.Spit;
import static org.firstinspires.ftc.teamcode.Core.Module.Intake.IntakeMotor.States.Wait;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Core.Hardware.HighCamera;
import org.firstinspires.ftc.teamcode.Core.Module.Others.BrakePiston;
import org.firstinspires.ftc.teamcode.Core.Module.Outtake.LinkageCamera;
import org.firstinspires.ftc.teamcode.Core.Robot;

@TeleOp(name = "Camera Align Test")
@Config
public class CameraAlignTest extends LinearOpMode {
    Robot robot;

    public enum LaunchZone{
        Far,
        Close
    }

    LaunchZone zone = LaunchZone.Far;

    boolean rumbled = false;
    public static double cameraReactivity = 0.02;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(1300);
        robot = new Robot(hardwareMap ,finalAutoPose, false , autoColor, telemetry, gamepad1);
        finalAutoPose = new Pose(6,6,0);
        robot.outtake.startBreakBeamThread();
        Constants.Globals.afterAuto = false;
        gamepad1.setLedColor(132 / 255.0, 88 / 255.0, 164 / 255.0, 2147483647);
        gamepad2.setLedColor(132 / 255.0, 88 / 255.0, 164 / 255.0, 2147483647);
        robot.outtake.linkageCamera.setState(LinkageCamera.States.Artifact);
        robot.camera.setPipeline(HighCamera.Pipelines.AprilTagLocation);
        FtcDashboard.getInstance().startCameraStream(robot.camera.ll, 0);
        waitForStart();
        while(opModeIsActive()){
            if (gamepad2.rightBumperWasPressed()) {
                robot.setAction(Robot.Actions.Shoot);
                robot.holdingSequence = true;
                rumbled = true;
            }
            if(gamepad2.rightBumperWasReleased()){
                robot.holdingSequence = false;
            }

            if(gamepad1.rightBumperWasPressed()){
                robot.brake.setState(BrakePiston.States.Brake);
            }

            if(gamepad1.leftBumperWasPressed()){
                robot.brake.setState(BrakePiston.States.Float);
            }

            if(gamepad2.left_bumper){
                if(robot.outtake.distanceToGoal >= 50){
                    robot.outtake.setShootingVelocity();
                }
            }

            if(zone == LaunchZone.Close){
                if(robot.outtake.distanceToGoal >= 50 && robot.outtake.distanceToGoal <= 250){
                    robot.outtake.setShootingVelocity();
                } else if(robot.outtake.distanceToGoal < 50){
                    robot.outtake.setShootingVelocity(50);
                }
            } else {
                if(robot.outtake.distanceToGoal >= 250 && robot.outtake.distanceToGoal <= 415){
                    robot.outtake.setShootingVelocity();
                } else if(robot.outtake.distanceToGoal > 415){
                    robot.outtake.setShootingVelocity(415);
                }
            }
            if (gamepad2.left_trigger >= 0.6){
                robot.outtake.openBlocker();
            }
            if (gamepad2.right_trigger >= 0.6){
                robot.outtake.closeBlocker();
            }

            if (gamepad2.dpad_left) {
                robot.outtake.offsetTurretToLeft(2.5);
            }
            if (gamepad2.dpad_right) {
                robot.outtake.offsetTurretToRight(2.5);
            }
            if (gamepad2.dpadUpWasPressed()) {
                robot.outtake.turret.setTarget(0);
                robot.outtake.turret.setOffset(0);
                robot.shouldAlignTurret = false;
            }
            if (gamepad2.dpadDownWasPressed()) {
                robot.shouldAlignTurret = true;
            }
            if(gamepad1.left_trigger >= 0.8){
                robot.intake.setPower(Spit);
            } else if(gamepad1.right_trigger >= 0.8){
                robot.intake.setPower(Collect);
            } else if(robot.intake.canStop) {
                robot.intake.setPower(Wait);
            }
            if(robot.lastAction == Robot.Actions.Shoot && rumbled){
                gamepad1.rumble(200);
                rumbled = false;
            }
            if(gamepad2.squareWasPressed()){
                robot.enableResetWithCamera();
            }
            if(gamepad2.circleWasPressed()){
                robot.disableResetWithCamera();
            }
            if(gamepad1.psWasPressed()){
                if(gamepad1.dpad_up) {
                    if (robot.allianceColor == Constants.Color.Blue) {
                        robot.drive.startFieldCentricDrive(gamepad1, true, 0);
                    }
                    robot.drive.setStartingPose(new Pose(6, 6, 0));
                    robot.drive.setPose(new Pose(6, 6, 0));
                    robot.outtake.turret.setOffset(0);
                }
                robot.drive.resetTeleOpHeading();
            }

            if(gamepad1.dpad_left) {
                robot.drive.setPose(new Pose(RedGoalDistance.getX(),RedGoalDistance.getY(),robot.drive.getPose().getHeading()));
            }

            if(gamepad1.dpad_right) {
                robot.drive.setPose(new Pose(BlueGoalDistance.getX(),BlueGoalDistance.getY(),robot.drive.getPose().getHeading()));
            }

            if(gamepad1.dpad_down) {
                telemetry.addData("da", "da");
                robot.drive.setStartingPose(new Pose(132, 6, Math.PI));
                robot.drive.setPose(new Pose(132, 6, Math.PI));
                robot.outtake.turret.setOffset(0);
                if (robot.allianceColor == Constants.Color.Red) {
                    robot.drive.startFieldCentricDrive(gamepad1, true, 0);
                }
            }

            if(gamepad2.ps){
                robot.setAction(Robot.Actions.ResetTurretCamera);
            }
            if(gamepad2.crossWasPressed()){
                robot.outtake.turret.setOffset(0);
            }
            if(gamepad2.circle){
                robot.setAction(Robot.Actions.StopShoot);
            }
            if(gamepad2.triangleWasPressed()){
                if(robot.outtake.turret.allianceColor == Constants.Color.Blue){
                    robot.outtake.turret.allianceColor= Constants.Color.Red;
                } else {
                    robot.outtake.turret.allianceColor= Constants.Color.Blue;
                }
            }
            if(gamepad2.leftStickButtonWasPressed()){
                robot.outtake.turret.reset();
            }
            if(Math.abs(gamepad2.right_stick_x) >= 0.4){
                robot.outtake.turret.addOffsetDegrees(-0.3 * gamepad2.right_stick_x);
            }
            
            if(gamepad2.optionsWasPressed()){
                if(zone == LaunchZone.Far){
                    zone = LaunchZone.Close;
                    gamepad1.setLedColor(49 / 255.0, 155 / 255.0, 164 / 255.0 , 2147483647);
                    gamepad2.setLedColor(49 / 255.0, 155 / 255.0, 164 / 255.0 , 2147483647);
                } else {
                    zone = LaunchZone.Far;
                    gamepad1.setLedColor(132 / 255.0, 88 / 255.0, 164 / 255.0, 2147483647);
                    gamepad2.setLedColor(132 / 255.0, 88 / 255.0, 164 / 255.0, 2147483647);
                }
            }
            if(gamepad2.shareWasPressed()){
                robot.setAction(Robot.Actions.StopCamera);
            }

            robot.outtake.turret.visionKp = cameraReactivity;
            telemetry.addData("Distance:", robot.outtake.distanceToGoal);
            telemetry.addData("Camera Angle:", robot.camera.getHorizontalOffset());
            telemetry.addData("Pose:", robot.drive.getPose());
            telemetry.update();
            robot.update();
        }
        robot.outtake.stopBreakBeamThread();
    }
}
