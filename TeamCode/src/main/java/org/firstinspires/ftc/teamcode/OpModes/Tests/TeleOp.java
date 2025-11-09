package org.firstinspires.ftc.teamcode.OpModes.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Core.Module.Intake.Intake;
import org.firstinspires.ftc.teamcode.Core.Robot;

import java.util.HashMap;

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends LinearOpMode {

    public static double velobelo = 4;
    Robot robot;
    public HashMap<String , ElapsedTime> timers = new HashMap<>();

    Telemetry graph;


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
        graph =FtcDashboard.getInstance().getTelemetry();
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

            telemetry.addData("State intake:" , robot.intake.motorIntake.getState());
            telemetry.addData("Power intake:" , robot.intake.motorIntake.power);
            telemetry.addData("Shooter 1:" , robot.shooter.motorUp.getPower());
            telemetry.addData("Shooter 2:" , robot.shooter.motorDown.getPower());
            telemetry.addData("Up:" , robot.shooter.motorUp.getCurrentPosition());
            telemetry.addData("Down:" , robot.shooter.motorDown.getCurrentPosition());
            graph.addData("Current Velo" , robot.shooter.motorUp.getVelocity());
            graph.addData("Target Velo" , velobelo);
            robot.update();
            telemetry.update();
            graph.update();
        }
    }
}
