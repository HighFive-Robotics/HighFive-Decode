package org.firstinspires.ftc.teamcode.OpModes.TeleOps;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Core.Module.Intake.Intake;
import org.firstinspires.ftc.teamcode.Core.Module.Intake.Joint;
import org.firstinspires.ftc.teamcode.Core.Module.Intake.MotorIntake;
import org.firstinspires.ftc.teamcode.Core.Module.Outtake.BlockerOuttake;
import org.firstinspires.ftc.teamcode.Core.Robot;

import java.util.HashMap;
import java.util.Optional;

@Config
@TeleOp(name = "💥TeleOp💥")
@SuppressWarnings("All")
public class TeleOpWow extends LinearOpMode {

    public static double littleVelo = 4 , bigVelo = 7.5 , negativeVelo = -2;


    public boolean intakeDriver2 = false;
    Robot robot;

    public HashMap<String , ElapsedTime> timers = new HashMap<>();

    Telemetry graph;


    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap , new Pose(0,0,0) , false , Constants.Color.Blue , telemetry,gamepad1);
        timers.put("leftStick1" , new ElapsedTime());
        timers.put("rightStick1" , new ElapsedTime());
        timers.put("rightBumper1" , new ElapsedTime());
        timers.put("rightBumper2" , new ElapsedTime());
        timers.put("leftBumper2" , new ElapsedTime());
        timers.put("leftTrigger2" , new ElapsedTime());
        timers.put("rightTrigger2" , new ElapsedTime());
        timers.put("square2" , new ElapsedTime());
        timers.put("circle2" , new ElapsedTime());
        timers.put("cross2" , new ElapsedTime());
        timers.put("triangle2" , new ElapsedTime());

        timers.get("leftStick1").reset();
        timers.get("rightStick1").reset();
        timers.get("rightBumper1").reset();
        timers.get("rightBumper2").reset();
        timers.get("leftBumper2").reset();
        timers.get("leftTrigger2").reset();
        timers.get("rightTrigger2").reset();
        timers.get("circle2").reset();
        timers.get("cross2").reset();
        timers.get("square2").reset();
        timers.get("triangle2").reset();

        graph = FtcDashboard.getInstance().getTelemetry();
        waitForStart();

        while (opModeIsActive()) {
            robot.teleOpDrive.driveFieldCentric(gamepad1, 0);

            if(gamepad1.left_trigger >= 0.8){
                robot.intake.setAction(Intake.IntakeActions.Spit);
            } else if(gamepad1.right_trigger >= 0.8){
                robot.intake.setAction(Intake.IntakeActions.Collect);
            } else if(!intakeDriver2) {
                robot.intake.setAction(Intake.IntakeActions.Wait);
            }

            if(gamepad1.right_bumper && timers.get("rightBumper").milliseconds() >= 250){
                gamepad2.rumble(150);
                timers.get("rightBumper").reset();
            }

            if(gamepad1.left_stick_button && timers.get("leftStick1").milliseconds() >= 250){
                robot.intake.joint.setState(Joint.States.Block);
                timers.get("leftStick1").reset();
            }

            if(gamepad1.right_stick_button && timers.get("rightStick1").milliseconds() >= 250){
                robot.intake.joint.setState(Joint.States.Pass);
                timers.get("rightStick1").reset();
            }

            if(gamepad1.triangle && gamepad1.dpad_up){
                robot.lift.setPower(1);
            } else {
                robot.lift.setPower(0);
            }

            if(gamepad2.right_bumper && timers.get("rightBumper2").milliseconds() >= 250){
                intakeDriver2 = true;
                robot.intake.motorIntake.setState(MotorIntake.States.Transfer);
                timers.get("rightBumper2").reset();
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
            }
            if(gamepad2.triangle && timers.get("triangle2").milliseconds() >= 250){
                robot.shooter.setTargetVelocity(bigVelo);
                timers.get("triangle2").reset();
            }
            if(gamepad1.ps){
                robot.drive.resetTeleOpHeading();
            }

            telemetry.addData("State intake:" , robot.intake.motorIntake.getState());
            telemetry.addData("Power intake:" , robot.intake.motorIntake.power);
            telemetry.addData("Shooter 1:" , robot.shooter.motorUp.getPower());
            telemetry.addData("Shooter 2:" , robot.shooter.motorDown.getPower());
            telemetry.addData("Up:" , robot.shooter.motorUp.getCurrentPosition());
            telemetry.addData("Down:" , robot.shooter.motorDown.getCurrentPosition());
            graph.addData("Current Velo" , robot.shooter.motorUp.getCurrentVelocity());
            telemetry.update();
            graph.update();
            robot.update();
        }
    }
}
