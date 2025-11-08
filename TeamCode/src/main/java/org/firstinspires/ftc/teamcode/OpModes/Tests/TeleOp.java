package org.firstinspires.ftc.teamcode.OpModes.Tests;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Core.Module.Intake.Intake;
import org.firstinspires.ftc.teamcode.Core.Robot;

import java.util.HashMap;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends LinearOpMode {

    public static double velobelo = 4;
    Robot robot;
    public HashMap<String , ElapsedTime> timers = new HashMap<>();

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap , new Pose(0,0,0) , false , Constants.Color.Blue , telemetry);
        timers.put("circle" , new ElapsedTime());
        timers.put("cross" , new ElapsedTime());
        timers.put("rightTrigger" , new ElapsedTime());
        timers.put("leftTrigger" , new ElapsedTime());
        timers.get("circle").reset();
        timers.get("cross").reset();
        timers.get("rightTrigger").reset();
        timers.get("leftTrigger").reset();
        waitForStart();

        while (opModeIsActive()) {
            robot.teleOpDrive.drive(gamepad1);

            if(gamepad1.cross && timers.get("cross").milliseconds() >= 250){
                robot.shooter.setTargetVelocity(velobelo);
                timers.get("cross").reset();
            }
            if(gamepad1.circle && timers.get("circle").milliseconds() >= 250){
                robot.shooter.setTargetVelocity(0);
                timers.get("circle").reset();
            }
            if(gamepad1.left_trigger >= 0.8){
                robot.intake.setAction(Intake.IntakeActions.Spit);
            } else if(gamepad1.right_trigger >= 0.8){
                robot.intake.setAction(Intake.IntakeActions.Collect);
            } else {
                robot.intake.setAction(Intake.IntakeActions.Wait);
            }

            robot.update();
            telemetry.update();
        }
    }
}
