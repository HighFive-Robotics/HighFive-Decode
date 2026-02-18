package org.firstinspires.ftc.teamcode.OpModes.TeleOps;

import static org.firstinspires.ftc.teamcode.Constants.Globals.autoColor;
import static org.firstinspires.ftc.teamcode.Constants.Globals.finalAutoPose;
import static org.firstinspires.ftc.teamcode.Core.Module.Intake.IntakeMotor.States.Collect;
import static org.firstinspires.ftc.teamcode.Core.Module.Intake.IntakeMotor.States.Spit;
import static org.firstinspires.ftc.teamcode.Core.Module.Intake.IntakeMotor.States.Wait;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Core.Robot;

import java.util.ArrayList;
import java.util.Arrays;

@TeleOp
public class TeleOpMaxVerstapen extends LinearOpMode {

    Robot robot;

    int i = 0;
//    double []velocityDown = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    ArrayList<Double> velocityDown = new ArrayList<>();
    boolean rumbled = false;
    boolean dynamicUpdate = false;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap ,finalAutoPose, false , autoColor , telemetry,gamepad1);
        finalAutoPose = new Pose();

        Constants.Globals.afterAuto = false;

        waitForStart();
        while(opModeIsActive()){
            if (gamepad2.rightBumperWasPressed()) {
                robot.setAction(Robot.Actions.Shoot);
            }
            if(gamepad2.rightBumperWasReleased()){
                robot.holdingSequence = false;
            }

            if(gamepad2.left_bumper){
                robot.outtake.setShootingVelocity();
            }

            if (gamepad2.left_trigger >= 0.6){
                robot.outtake.openBlocker();
            }
            if (gamepad2.right_trigger >= 0.6){
                robot.outtake.closeBlocker();
            }

            if (gamepad2.dpadLeftWasPressed()) {
                robot.outtake.offsetTurretToLeft(2.5);
            }
            if (gamepad1.dpadRightWasPressed()) {
                robot.outtake.offsetTurretToRight(2.5);
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

            if(gamepad1.optionsWasPressed()){
                dynamicUpdate = !dynamicUpdate;
            }

            if(dynamicUpdate){
                robot.outtake.setShootingVelocityOffset(-2);
            }

            if(robot.outtake.hasShot){
                velocityDown.add(robot.outtake.shooter.velocityDown);
                robot.outtake.hasShot = false;
                i++;
            }

            robot.outtake.debug();
            telemetry.addData("Velocity Down:", velocityDown.toString());
            telemetry.addData("Shoot :", robot.outtake.hasShot);
            telemetry.update();
            robot.update();
        }
    }
}
