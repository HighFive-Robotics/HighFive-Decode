package org.firstinspires.ftc.teamcode.OpModes.Tests;

import static org.firstinspires.ftc.teamcode.Constants.DeviceNames.intakeMotorName;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Core.Module.Outtake.Shooter;

@Config
@TeleOp(name = "Shooter Calibration", group = "Tests")
public class ShooterCalibration extends LinearOpMode {
    enum Mode{
        Up,
        Down,
        Both,
        Angle,
        None
    }
    public static Mode mode = Mode.None;
    DcMotorEx motor;
    public static double targetVelocity = 0, angle = 32 ,sV=0;
    private Shooter shooter;
    Follower drive;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        shooter = new Shooter(hardwareMap);
        drive = Constants.createFollower(hardwareMap);
        drive.setStartingPose(new Pose(72,72,Math.PI));
        drive.startFieldCentricDrive(gamepad1, true, 0);
        motor = hardwareMap.get(DcMotorEx.class, intakeMotorName);
        telemetry.addLine("Init");
        waitForStart();
        while (opModeIsActive()){
            switch (mode) {
                case Down:
                    shooter.setDownTargetVelocity(targetVelocity);
                    shooter.nanUp();
                    shooter.updateCoefDown();

                    break;
                case Up:
                    shooter.setUpTargetVelocity(targetVelocity);
                    shooter.nanDown();
                    shooter.updateCoefUp();

                    break;
                case Both:
                    shooter.setTargetVelocity(targetVelocity, sV);
                    shooter.updateAllCoef();

                    break;
                case Angle:
                    shooter.setAntiBackSpinVelocity(targetVelocity);
                    shooter.updateAllCoef();

                    break;

            }
            if(gamepad1.aWasPressed())shooter.setAntiBackSpinVelocity(2.5);
            if(gamepad1.bWasPressed())shooter.setAntiBackSpinVelocity(2.8);
            if(gamepad1.xWasPressed())shooter.setAntiBackSpinVelocity(3.9);
            if(gamepad1.yWasPressed())shooter.setAntiBackSpinVelocity(0);
            if(gamepad1.dpadDownWasPressed())shooter.setAntiBackSpinVelocity(4);
            if(gamepad1.leftBumperWasPressed()) shooter.closeBlocker();
            if(gamepad1.rightBumperWasPressed()) shooter.openBlocker();
            if(gamepad1.psWasPressed()) drive.resetTeleOpHeading();
            if(gamepad1.right_trigger >= 0.4){
                motor.setPower(1);
            }else if(gamepad1.left_trigger >= 0.4){
                motor.setPower(-1);
            }else motor.setPower(0);
            telemetry.addData("Target Velocity",targetVelocity);
            telemetry.addData("Velocity Up",shooter.motorUp.getCurrentVelocity());
            telemetry.addData("Velocity Down",shooter.motorDown.getCurrentVelocity());
            telemetry.addData("Power", shooter.motorDown.getPower());
            shooter.update();
            drive.update();
            telemetry.update();
        }
    }
}