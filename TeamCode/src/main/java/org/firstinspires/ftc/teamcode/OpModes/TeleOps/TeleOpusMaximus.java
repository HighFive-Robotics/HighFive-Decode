package org.firstinspires.ftc.teamcode.OpModes.TeleOps;

import static org.firstinspires.ftc.teamcode.Constants.Globals.afterAuto;
import static org.firstinspires.ftc.teamcode.Constants.Globals.autoColor;
import static org.firstinspires.ftc.teamcode.Constants.Globals.finalAutoPose;
import static org.firstinspires.ftc.teamcode.Constants.Intake.SorterConstants.sorterColors;
import static org.firstinspires.ftc.teamcode.Core.Module.Intake.IntakeMotor.States.Wait;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Core.Module.Intake.Intake;
import org.firstinspires.ftc.teamcode.Core.Module.Intake.IntakeMotor;
import org.firstinspires.ftc.teamcode.Core.Module.Intake.Sorter;
import org.firstinspires.ftc.teamcode.Core.Module.Outtake.BlockerOuttake;
import org.firstinspires.ftc.teamcode.Core.Robot;

import java.util.HashMap;

@Config
@TeleOp(name = "💥TeleOp💥")
@SuppressWarnings("All")
public class TeleOpusMaximus extends LinearOpMode {

    public static double littleVelo = 4 , bigVelo = 4.8 , negativeVelo = -2;
    public boolean intakeDriver2 = false, rumbled = false;
    Robot robot;
    public HashMap<String , ElapsedTime> timers = new HashMap<>();
    ElapsedTime loopTimer = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap ,finalAutoPose, false , autoColor , telemetry,gamepad1);

        finalAutoPose = new Pose();
        timers.put("rightBumper1" , new ElapsedTime());
        timers.put("leftBumper1" , new ElapsedTime());
        timers.put("rightBumper2" , new ElapsedTime());
        timers.put("leftTrigger2" , new ElapsedTime());
        timers.put("rightTrigger2" , new ElapsedTime());
        timers.put("square2" , new ElapsedTime());
        timers.put("circle2" , new ElapsedTime());
        timers.put("triangle2" , new ElapsedTime());
        timers.put("rumble" , new ElapsedTime());

        timers.get("rightBumper1").reset();
        timers.get("leftBumper1").reset();
        timers.get("rightBumper2").reset();
        timers.get("leftTrigger2").reset();
        timers.get("rightTrigger2").reset();
        timers.get("circle2").reset();
        timers.get("square2").reset();
        timers.get("triangle2").reset();
        timers.get("rumble").reset();

        robot.camera.stopCapture();
        Constants.Globals.afterAuto = false;
        robot.intake.setCollectType(Intake.CollectTypes.Normal);
        robot.intake.setState(Intake.States.Wait);
        gamepad1.setLedColor(49 / 255.0, 155 / 255.0, 164 / 255.0 , 2147483647);
        gamepad2.setLedColor(49 / 255.0, 155 / 255.0, 164 / 255.0 , 2147483647);
        waitForStart();
        while (opModeIsActive()) {

            switch(robot.intake.getCollectType()){
                case Sorted:
                    if(gamepad2.square && timers.get("square2").milliseconds() >= 250){
                        robot.shooter.setTargetVelocity(robot.shooter.getVelocityFromDistance(robot.getDistance()));
                        timers.get("square2").reset();
                        timers.get("rumble").reset();
                        rumbled = true;
                    }
                    if(gamepad2.dpadLeftWasPressed()){
                        robot.intake.setAction(Intake.Actions.PreviousSlot);
                    }
                    if(gamepad2.dpadRightWasPressed()){
                        robot.intake.setAction(Intake.Actions.NextSlot);
                    }
                    if(gamepad2.leftBumperWasReleased()){
                        sorterColors = new Constants.Color[]{Constants.Color.None, Constants.Color.None, Constants.Color.None};
                        robot.intake.setState(Intake.States.Collect);
                    }
                    if(gamepad2.triangleWasPressed()){
                        robot.shooter.setTargetVelocity(bigVelo);
                        timers.get("rumble").reset();
                        rumbled = true;
                    }
                    if(gamepad2.rightBumperWasPressed()){
                        robot.setAction(Robot.Actions.ShootFast);
                    }
                    if(gamepad2.dpadUpWasPressed()){
                        robot.setAction(Robot.Actions.AddGreenToQueue);
                    }
                    if(gamepad2.dpadDownWasPressed()){
                        robot.setAction(Robot.Actions.AddPurpleToQueue);
                    }
                    break;
                case Normal:
                    if(gamepad1.leftBumperWasPressed()){
                        robot.setAction(Robot.Actions.PrepareForShooting);
                        timers.get("leftBumper1").reset();
                    }
                    if(gamepad2.rightBumperWasPressed()){
                        intakeDriver2 = true;
                        robot.setAction(Robot.Actions.ShootFastNormal);
                        timers.get("rightBumper2").reset();
                        timers.get("rumble").reset();
                        rumbled = true;
                    }
                    if(gamepad2.leftBumperWasPressed()){
                        intakeDriver2 = false;
                        robot.intake.setPower(Wait);
                    }
                    if(robot.intake.getPower() == Wait){
                        intakeDriver2 = false;
                    }
                    if(gamepad2.squareWasPressed()){
                        robot.shooter.setTargetVelocity(negativeVelo);
                    }
                    if(gamepad2.triangle && timers.get("triangle2").milliseconds() >= 250){
                        robot.shooter.setTargetVelocity(robot.shooter.getVelocityFromDistance(robot.getDistance()));
                        timers.get("triangle2").reset();
                        timers.get("rumble").reset();
                        rumbled = true;
                    }
                    break;
                case Mix:
                    if(gamepad1.left_bumper && timers.get("leftBumper1").milliseconds() >= 250){
                        robot.setAction(Robot.Actions.PrepareForShooting);
                        timers.get("leftBumper1").reset();
                    }
                    if(gamepad2.right_bumper && timers.get("rightBumper2").milliseconds() >= 250){
                        intakeDriver2 = true;
                        robot.setAction(Robot.Actions.Shoot);
                        timers.get("rightBumper2").reset();
                        timers.get("rumble").reset();
                        rumbled = true;
                    }
                    if(gamepad2.leftBumperWasPressed()){
                        intakeDriver2 = false;
                        robot.intake.setPower(Wait);
                    }
                    if(robot.intake.getPower() == Wait){
                        intakeDriver2 = false;
                    }
                    if(gamepad2.square && timers.get("square2").milliseconds() >= 250){
                        robot.shooter.setTargetVelocity(robot.shooter.getVelocityFromDistance(robot.getDistance()));
                        timers.get("square2").reset();
                        timers.get("rumble").reset();
                        rumbled = true;
                    }
                    if(gamepad2.dpadLeftWasPressed()){
                        robot.intake.setAction(Intake.Actions.PreviousSlot);
                    }
                    if(gamepad2.dpadRightWasPressed()){
                        robot.intake.setAction(Intake.Actions.NextSlot);
                    }
                    if(gamepad2.leftBumperWasReleased()){
                        sorterColors = new Constants.Color[]{Constants.Color.None, Constants.Color.None, Constants.Color.None};
                        robot.intake.setState(Intake.States.Collect);
                    }
                    if(gamepad2.triangleWasPressed()){
                        robot.shooter.setTargetVelocity(bigVelo);
                        timers.get("rumble").reset();
                        rumbled = true;
                    }
                    break;
            }

            if(gamepad1.left_trigger >= 0.8){
                robot.intake.setPower(IntakeMotor.States.Spit);
            } else if(gamepad1.right_trigger >= 0.8){
                robot.intake.setPower(IntakeMotor.States.Collect);
            } else if(!intakeDriver2 && robot.intake.canStop) {
                robot.intake.setPower(Wait);
            }

            if(gamepad1.right_bumper && timers.get("rightBumper1").milliseconds() >= 250){
                gamepad2.rumble(150);
                timers.get("rightBumper1").reset();
            }

            if(gamepad2.left_trigger >= 0.7 && timers.get("leftTrigger2").milliseconds() >= 250){
                robot.shooter.blocker.setState(BlockerOuttake.States.Open);
                timers.get("leftTrigger2").reset();
            }

            if(gamepad2.right_trigger >= 0.7 && timers.get("rightTrigger2").milliseconds() >= 250){
                robot.shooter.blocker.setState(BlockerOuttake.States.Close);
                timers.get("rightTrigger2").reset();
            }

            if(gamepad2.circle && timers.get("circle2").milliseconds() >= 250){
                robot.shooter.setTargetVelocity(0);
                timers.get("circle2").reset();
            }

            if(gamepad2.crossWasPressed()){
                robot.shooter.setTargetVelocity(littleVelo);
                timers.get("rumble").reset();
                rumbled = true;
            }

            if(gamepad2.crossWasPressed()){
                robot.shooter.setTargetVelocity(littleVelo);
                timers.get("rumble").reset();
                rumbled = true;
            }

            if(gamepad2.rightStickButtonWasPressed()){
                robot.intake.sorter.setState(Sorter.States.Automated);
            }

            if(gamepad2.leftStickButtonWasPressed()){
                robot.intake.sorter.setState(Sorter.States.Manual);
            }

            if(gamepad2.psWasPressed()){
                if(robot.intake.getCollectType() != Intake.CollectTypes.Normal){
                    robot.intake.setCollectType(Intake.CollectTypes.Normal);
                    robot.intake.setState(Intake.States.Wait);
                    gamepad1.setLedColor(49 / 255.0, 155 / 255.0, 164 / 255.0 , 2147483647);
                    gamepad2.setLedColor(49 / 255.0, 155 / 255.0, 164 / 255.0 , 2147483647);
                } else {
                    robot.intake.setCollectType(Intake.CollectTypes.Sorted);
                    robot.intake.setState(Intake.States.Collect);
                    gamepad1.setLedColor(132 / 255.0, 88 / 255.0, 164 / 255.0, 2147483647);
                    gamepad2.setLedColor(132 / 255.0, 88 / 255.0, 164 / 255.0, 2147483647);
                }
            }

            if(gamepad1.psWasPressed()){
                if(gamepad1.dpad_up) {
                    if (robot.allianceColor == Constants.Color.Blue) {
                        robot.drive.startFieldCentricDrive(gamepad1, true, 0);
                    }
                    robot.drive.setStartingPose(new Pose(8, 8, 0));
                    robot.drive.setPose(new Pose(8, 8, 0));
                    robot.allianceColor = Constants.Color.Red;
                }
                robot.drive.resetTeleOpHeading();
            }

            if(robot.shooter.atTarget() && robot.shooter.getTarget() >= 2 && rumbled && timers.get("rumble").milliseconds() >= 500){
                gamepad2.rumble(200);
                rumbled = false;
            }

            telemetry.addData("Final Auto Pose Heading" , finalAutoPose.getHeading());
            telemetry.addData("Color 1:", sorterColors[0]);
            telemetry.addData("Color 2:", sorterColors[1]);
            telemetry.addData("Color 3:", sorterColors[2]);
            telemetry.addData("Update count" , robot.intake.count);
            telemetry.addData("Intake Motor State" , robot.intake.getState());
            telemetry.addData("Pose", robot.drive.getPose().toString());
            telemetry.addData("Hz", 1.0 / loopTimer.seconds());
            loopTimer.reset();
            telemetry.update();
            robot.update();
        }
    }
}
