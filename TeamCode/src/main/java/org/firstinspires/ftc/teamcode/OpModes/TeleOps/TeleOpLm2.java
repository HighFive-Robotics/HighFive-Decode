package org.firstinspires.ftc.teamcode.OpModes.TeleOps;

import static org.firstinspires.ftc.teamcode.Constants.Globals.RedGoal;
import static org.firstinspires.ftc.teamcode.Constants.Globals.finalAutoPose;
import static org.firstinspires.ftc.teamcode.Constants.Intake.SorterConstants.sorterColors;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Core.Module.Intake.Intake;
import org.firstinspires.ftc.teamcode.Core.Module.Intake.IntakeMotor;
import org.firstinspires.ftc.teamcode.Core.Module.Outtake.BlockerOuttake;
import org.firstinspires.ftc.teamcode.Core.Robot;

import java.util.HashMap;

@Config
@TeleOp(name = "💥TeleOp LM2💥")
@SuppressWarnings("All")
public class TeleOpLm2 extends LinearOpMode {

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
        robot.camera.startCapture();
        waitForStart();

        while (opModeIsActive()) {

            if(gamepad1.left_trigger >= 0.8){
                robot.intake.setPower(IntakeMotor.States.Spit);
            } else if(gamepad1.right_trigger >= 0.8){
                robot.intake.setPower(IntakeMotor.States.Collect);
            } else if(!intakeDriver2 && robot.intake.canStop) {
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
                robot.intake.setState(Intake.States.Wait);
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
                robot.shooter.setTargetVelocity(robot.shooter.getVelocityFromDistance(robot.getDistance()));
                timers.get("triangle2").reset();
                timers.get("rumble").reset();
                rumbled = true;
            }

            if(gamepad2.psWasPressed()){
                if(robot.intake.getCollectType() != Intake.CollectTypes.Normal){
                    robot.intake.setCollectType(Intake.CollectTypes.Normal);
                    robot.intake.setState(Intake.States.Wait);
                } else {
                    robot.intake.setCollectType(Intake.CollectTypes.Sorted);
                    robot.intake.setState(Intake.States.Collect);
                }
            }

            if(gamepad1.dpadUpWasPressed()){
                robot.setAction(Robot.Actions.ShootFast);
            }

            if(gamepad1.circleWasPressed()){
                robot.setAction(Robot.Actions.ShootGPP);
            }

            if(gamepad1.triangleWasPressed()){
                robot.setAction(Robot.Actions.ShootPGP);
            }

            if(gamepad1.squareWasPressed()){
                robot.setAction(Robot.Actions.ShootPPG);
            }

            if(gamepad1.dpadLeftWasPressed()){
                robot.intake.setAction(Intake.Actions.FindGreen);
            }
            if(gamepad1.dpadRightWasPressed()){
                robot.intake.setAction(Intake.Actions.FindPurple);
            }

            if(gamepad2.dpadLeftWasPressed()){
                robot.intake.setAction(Intake.Actions.PreviousSlot);
            }
            if(gamepad2.dpadRightWasPressed()){
                robot.intake.setAction(Intake.Actions.NextSlot);
            }
            if(gamepad1.psWasPressed()){
                robot.drive.resetTeleOpHeading();
            }
            if(robot.intake.getPower() == IntakeMotor.States.Wait){
                intakeDriver2 = false;
            }

            if(robot.shooter.atTarget() && robot.shooter.getTarget() >= 2 && rumbled && timers.get("rumble").milliseconds() >= 500){
                gamepad2.rumble(200);
                rumbled = false;
            }

            if(gamepad2.dpadDownWasPressed()){
                sorterColors = new Constants.Color[]{Constants.Color.None, Constants.Color.None, Constants.Color.None};
                robot.intake.setState(Intake.States.Collect);
            }

            telemetry.addData("Color 1:", sorterColors[0]);
            telemetry.addData("Color 2:", sorterColors[1]);
            telemetry.addData("Color 3:", sorterColors[2]);
//            telemetry.addData("Shooter error velo", robot.shooter.getVelocityError());
//            telemetry.addData("Shooter target", robot.shooter.getTarget());
//            telemetry.addData("Shooter velocity", robot.shooter.motorUp.getCurrentVelocity());
//            telemetry.addData("Robot pose", robot.drive.getPose().toString());
//            telemetry.addData("Goal Red", RedGoal.toString());
//            telemetry.addData("Dist Red", robot.getDistance());
            telemetry.addData("Update count" , robot.intake.count);
            telemetry.addData("Intake Motor State" , robot.intake.getState());
            telemetry.addData("Hz", 1.0 / loopTimer.seconds());

            loopTimer.reset();
            telemetry.update();
            robot.update();
        }
    }
}
