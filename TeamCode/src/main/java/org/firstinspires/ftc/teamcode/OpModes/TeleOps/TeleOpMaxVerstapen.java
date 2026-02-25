package org.firstinspires.ftc.teamcode.OpModes.TeleOps;

import static org.firstinspires.ftc.teamcode.Constants.Globals.autoColor;
import static org.firstinspires.ftc.teamcode.Constants.Globals.finalAutoPose;
import static org.firstinspires.ftc.teamcode.Core.Module.Intake.IntakeMotor.States.Collect;
import static org.firstinspires.ftc.teamcode.Core.Module.Intake.IntakeMotor.States.Spit;
import static org.firstinspires.ftc.teamcode.Core.Module.Intake.IntakeMotor.States.Wait;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Core.Module.Outtake.LinkageCamera;
import org.firstinspires.ftc.teamcode.Core.Robot;

import java.util.ArrayList;
import java.util.Arrays;

@TeleOp(name = "😎TeleOp😎")
public class TeleOpMaxVerstapen extends LinearOpMode {

    Robot robot;

    public enum LaunchZone{
        Far,
        Close
    }

    LaunchZone zone = LaunchZone.Far;

    boolean rumbled = false;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new Robot(hardwareMap ,finalAutoPose, false , autoColor, telemetry, gamepad1);
        finalAutoPose = new Pose(6,6,0);
        robot.outtake.startBreakBeamThread();
        Constants.Globals.afterAuto = false;
        gamepad1.setLedColor(132 / 255.0, 88 / 255.0, 164 / 255.0, 2147483647);
        gamepad2.setLedColor(132 / 255.0, 88 / 255.0, 164 / 255.0, 2147483647);
        robot.outtake.linkageCamera.setState(LinkageCamera.States.Artifact);
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
                }
                robot.drive.resetTeleOpHeading();
            }
            if(gamepad2.ps){
                robot.setAction(Robot.Actions.ResetTurretCamera);
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
            telemetry.addData("Distance:", robot.outtake.distanceToGoal);
            telemetry.addData("Bleah:",robot.outtake.distanceToGoal >= 50 && robot.outtake.distanceToGoal <= 250);
            telemetry.addData("Bleah2:",robot.outtake.distanceToGoal >= 250 && robot.outtake.distanceToGoal <= 415);
            telemetry.update();
            robot.update();
        }
        robot.outtake.stopBreakBeamThread();
    }
}
