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

    int i = 0;
    ArrayList<Double> velocityDown = new ArrayList<>();
    boolean rumbled = false;
    boolean dynamicUpdate = false;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new Robot(hardwareMap ,finalAutoPose, false , autoColor, telemetry, gamepad1);
        finalAutoPose = new Pose();
        robot.outtake.startBreakBeamThread();
        Constants.Globals.afterAuto = false;

        gamepad1.setLedColor(132 / 255.0, 88 / 255.0, 164 / 255.0, 2147483647);
        gamepad2.setLedColor(132 / 255.0, 88 / 255.0, 164 / 255.0, 2147483647);
        waitForStart();
        while(opModeIsActive()){
            if (gamepad2.rightBumperWasPressed()) {
                robot.setAction(Robot.Actions.Shoot);
                robot.holdingSequence = true;
            }
            if(gamepad2.rightBumperWasReleased()){
                robot.holdingSequence = false;
            }

            if(gamepad2.left_bumper){
                robot.outtake.setShootingVelocity();
            }

            if(zone == LaunchZone.Close){
                rumbled = true;
                if(robot.outtake.distanceToGoal <= 250){
                    robot.outtake.setShootingVelocity();
                }
            } else {
                if(robot.outtake.distanceToGoal >= 300){
                    rumbled = true;
                    robot.outtake.setShootingVelocity();
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

            if(robot.outtake.atTarget() && rumbled){
                gamepad2.rumble(200);
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

            /*if(dynamicUpdate){
                robot.outtake.setShootingVelocityOffset(-2);
            }*/

            if(gamepad2.ps){
                robot.resetWithCamera();
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

            robot.outtake.debug();
            telemetry.addData("Shoot BreakBeam :", robot.outtake.hasShot);
            telemetry.update();
            robot.update();
        }
        robot.outtake.stopBreakBeamThread();
    }
}
