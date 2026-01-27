package org.firstinspires.ftc.teamcode.OpModes.Tests;

import static org.firstinspires.ftc.teamcode.Constants.DeviceNames.intakeMotorName;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
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
    }
    public static Mode mode = Mode.Down;
    DcMotorEx motor;
    public static double targetVelocity = 0;
    private Shooter shooter;
    Follower drive;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        shooter = new Shooter(hardwareMap);
        drive = Constants.createFollower(hardwareMap);
        drive.startFieldCentricDrive(gamepad1, true, 0);
        motor = hardwareMap.get(DcMotorEx.class, intakeMotorName);
        telemetry.addLine("Init");
        waitForStart();
        while (opModeIsActive()){
            switch (mode){
                case Down:
                    shooter.setDownTargetVelocity(targetVelocity);
                    shooter.nanUp();
                    shooter.updateCoefDown();
                    shooter.update();
                    break;
                case Up:
                    shooter.setUpTargetVelocity(targetVelocity);
                    shooter.nanDown();
                    shooter.updateCoefUp();
                    shooter.update();
                    break;
                case Both:
                    shooter.setFullTargetVelocity(targetVelocity);
                    shooter.updateAllCoef();
                    shooter.update();
                    break;
            }
            if(gamepad1.aWasPressed()) mode = Mode.Both;
            if(gamepad1.bWasPressed()) mode = Mode.Down;
            if(gamepad1.xWasPressed()) mode = Mode.Up;
            if(gamepad1.right_trigger >= 0.4){
                motor.setPower(1);
            }else motor.setPower(0);

            telemetry.addData("Target Velocity",targetVelocity);
            telemetry.addData("Velocity Up",shooter.motorUp.getCurrentVelocity());
            telemetry.addData("Velocity Down",shooter.motorDown.getCurrentVelocity());
            telemetry.addData("Power", shooter.motorDown.getPower());
            telemetry.update();
        }
    }
}