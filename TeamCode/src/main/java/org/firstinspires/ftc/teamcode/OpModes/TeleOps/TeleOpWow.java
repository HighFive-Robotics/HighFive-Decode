package org.firstinspires.ftc.teamcode.OpModes.TeleOps;

import static org.firstinspires.ftc.teamcode.Constants.Globals.finalAutoPose;
import static org.firstinspires.ftc.teamcode.Constants.Intake.SorterConstants.artifactNumber;
import static org.firstinspires.ftc.teamcode.Constants.Intake.SorterConstants.greenArtifactNumber;
import static org.firstinspires.ftc.teamcode.Constants.Intake.SorterConstants.kD;
import static org.firstinspires.ftc.teamcode.Constants.Intake.SorterConstants.kF;
import static org.firstinspires.ftc.teamcode.Constants.Intake.SorterConstants.kI;
import static org.firstinspires.ftc.teamcode.Constants.Intake.SorterConstants.kP;
import static org.firstinspires.ftc.teamcode.Constants.Intake.SorterConstants.purpleArtifactNumber;
import static org.firstinspires.ftc.teamcode.Constants.Intake.SorterConstants.sorterColors;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Core.Hardware.HighServo;
import org.firstinspires.ftc.teamcode.Core.Module.Intake.Intake;
import org.firstinspires.ftc.teamcode.Core.Module.Intake.IntakeMotor;
import org.firstinspires.ftc.teamcode.Core.Module.Outtake.BlockerOuttake;
import org.firstinspires.ftc.teamcode.Core.Robot;

import java.util.Arrays;
import java.util.HashMap;

@Config
@TeleOp(name = "💥TeleOp💥")
@SuppressWarnings("All")
public class TeleOpWow extends LinearOpMode {

    public static double littleVelo = 4 , bigVelo = 7.5 , negativeVelo = -2;


    public boolean intakeDriver2 = false, rumbled = false;
    Robot robot;

    public HashMap<String , ElapsedTime> timers = new HashMap<>();
    ElapsedTime loopTimer = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap ,finalAutoPose, false , Constants.Color.Blue , telemetry,gamepad1);
        finalAutoPose = new Pose();
        timers.put("leftStick1" , new ElapsedTime());
        timers.put("rightStick1" , new ElapsedTime());
        timers.put("rightBumper1" , new ElapsedTime());
        timers.put("leftBumper1" , new ElapsedTime());
        timers.put("rightBumper2" , new ElapsedTime());
        timers.put("leftBumper2" , new ElapsedTime());
        timers.put("leftTrigger2" , new ElapsedTime());
        timers.put("rightTrigger2" , new ElapsedTime());
        timers.put("square2" , new ElapsedTime());
        timers.put("circle2" , new ElapsedTime());
        timers.put("cross2" , new ElapsedTime());
        timers.put("triangle2" , new ElapsedTime());
        timers.put("rumble" , new ElapsedTime());

        timers.get("leftStick1").reset();
        timers.get("rightStick1").reset();
        timers.get("rightBumper1").reset();
        timers.get("leftBumper1").reset();
        timers.get("rightBumper2").reset();
        timers.get("leftBumper2").reset();
        timers.get("leftTrigger2").reset();
        timers.get("rightTrigger2").reset();
        timers.get("circle2").reset();
        timers.get("cross2").reset();
        timers.get("square2").reset();
        timers.get("triangle2").reset();
        timers.get("rumble").reset();

        waitForStart();

        while (opModeIsActive()) {

            if(gamepad1.left_trigger >= 0.8){
                robot.intake.setPower(IntakeMotor.States.Spit);
            } else if(gamepad1.right_trigger >= 0.8){
                robot.intake.setPower(IntakeMotor.States.Collect);
            } else if(!intakeDriver2) {
                robot.intake.setPower(IntakeMotor.States.Wait);
            }

            if(gamepad1.left_bumper && timers.get("leftBumper1").milliseconds() >= 250){
                robot.setAction(Robot.Actions.PrepareForShooting);
                timers.get("leftBumper1").reset();
            }

            if(gamepad1.right_bumper && timers.get("rightBumper1").milliseconds() >= 250){
                gamepad2.rumble(150);
                timers.get("rightBumper1").reset();
            }

            if(gamepad2.right_bumper && timers.get("rightBumper2").milliseconds() >= 250){
                intakeDriver2 = true;
                robot.setAction(Robot.Actions.Shoot);
                timers.get("rightBumper2").reset();
                timers.get("rumble").reset();
                rumbled = true;
            }

            if(gamepad2.left_bumper && timers.get("leftBumper2").milliseconds() >= 250){
                intakeDriver2 = false;
                robot.intake.setAction(Intake.IntakeActions.Wait);
                timers.get("leftBumper2").reset();
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
            if(gamepad2.square && timers.get("square2").milliseconds() >= 250){
                robot.shooter.setTargetVelocity(negativeVelo);
                timers.get("square2").reset();
            }
            if(gamepad2.cross && timers.get("cross2").milliseconds() >= 250){
                robot.shooter.setTargetVelocity(littleVelo);
                timers.get("cross2").reset();
                timers.get("rumble").reset();
                rumbled = true;
            }
            if(gamepad2.triangle && timers.get("triangle2").milliseconds() >= 250){
                robot.shooter.setTargetVelocity(bigVelo);
                timers.get("triangle2").reset();
                timers.get("rumble").reset();
                rumbled = true;
            }

            if(gamepad2.dpad_left && timers.get("square2").milliseconds() >= 250){
                robot.intake.sorter.setPreviousSlot();
                timers.get("square2").reset();
            }
            if(gamepad2.dpad_right && timers.get("square2").milliseconds() >= 250){
                robot.intake.sorter.setNextSlot();

                timers.get("square2").reset();
            }
            if(gamepad1.ps){
                 robot.drive.resetTeleOpHeading();
            }

            if(robot.intake.getLastAction() == Intake.IntakeActions.Wait){
                intakeDriver2 = false;
            }

            if(robot.shooter.atTarget() && robot.shooter.getTarget() >= 2 && rumbled && timers.get("rumble").milliseconds() >= 500){
                gamepad2.rumble(200);
                rumbled = false;
            }

            if(gamepad2.dpad_down){
                sorterColors = new Constants.Color[]{Constants.Color.None, Constants.Color.None, Constants.Color.None};
            }

    robot.intake.sorter.servo.setPIDCoefficients(kP,kI,kD,kF, HighServo.FeedForwardType.Arm,1);
            /*telemetry.addData("State intake:" , robot.intake.intakeMotor.getState());
            telemetry.addData("Power intake:" , robot.intake.intakeMotor.power);
            telemetry.addData("Shooter 1:" , robot.shooter.motorUp.getPower());
            telemetry.addData("Shooter 2:" , robot.shooter.motorDown.getPower());
            telemetry.addData("Up:" , robot.shooter.motorUp.getCurrentPosition());
            telemetry.addData("Down:" , robot.shooter.motorDown.getCurrentPosition());
            graph.addData("Current Velo" , robot.shooter.motorUp.getCurrentVelocity());
            telemetry.addData("Current Velo" , robot.shooter.motorUp.getCurrentVelocity());
            graph.addData("Current Velo From Current" , robot.shooter.motorUp.getVelocityFromCurrent(Constants.Globals.voltage));
            telemetry.addData("Current Velo From Current" , robot.shooter.motorUp.getVelocityFromCurrent(Constants.Globals.voltage));
            telemetry.addData("Rumble" , rumbled);
            telemetry.addData("target > 2" , robot.shooter.getTarget() > 2);
            telemetry.addData("at target" , robot.shooter.atTarget() );
            telemetry.addData("rumbled" , timers.get("rumble").milliseconds());

            telemetry.addData("Slot: ", robot.intake.sorter.getSlot());
            telemetry.addData("Slot number: ", robot.intake.sorter.slotNumber);
            telemetry.addData("Action intake:" , robot.intake.getLastAction());
            telemetry.addData("Collect type:" , robot.intake.getCollectType());
            telemetry.addData("Break beam:" , robot.intake.breakBeamCollected);
            telemetry.addData("ArtifactPassThrough:" , robot.intake.artifactPassThrough);
            telemetry.addData("Color:" , robot.intake.sensor.getColor());
            telemetry.addData("Colors", sorterColors.toString());
            telemetry.addData("Is Full", robot.intake.sorter.isFull);*/

            telemetry.addData("Angle:", robot.intake.sorter.servo.getCurrentPositionPID());
            telemetry.addData("Error:" , robot.intake.sorter.servo.pidfController.getPositionError());
            telemetry.addData("Target:" , robot.intake.sorter.servo.getTargetPID());
            telemetry.addData("Power:", robot.intake.sorter.servo.getPowerPID(robot.intake.sorter.servo.getCurrentPositionPID()));
            telemetry.addData("Velocity Error" , robot.shooter.getVelocityError());
            telemetry.addData("Color 1:", sorterColors[0]);
            telemetry.addData("Color 2:", sorterColors[1]);
            telemetry.addData("Color 3:", sorterColors[2]);
            telemetry.addData("Artifact number:", artifactNumber);
            telemetry.addData("Purple Artifact number:", purpleArtifactNumber);
            telemetry.addData("Green Artifact number:", greenArtifactNumber);
            robot.intake.sensor.telemetry(telemetry);
            telemetry.addData("Hz", 1.0 / loopTimer.seconds());
            loopTimer.reset();
            telemetry.update();
            robot.update();
        }
    }
}
