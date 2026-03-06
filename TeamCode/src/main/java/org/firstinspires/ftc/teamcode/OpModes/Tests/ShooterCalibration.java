package org.firstinspires.ftc.teamcode.OpModes.Tests;

import static org.firstinspires.ftc.teamcode.Constants.DeviceNames.intakeMotorName;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Core.Hardware.HighCamera;
import org.firstinspires.ftc.teamcode.Core.Module.Intake.IntakeMotor;
import org.firstinspires.ftc.teamcode.Core.Robot;

@Config
@TeleOp(name = "Shooter Calibration", group = "Tests")
public class ShooterCalibration extends LinearOpMode {

    public enum Mode {
        Manual,
        Physics
    }
    public static Mode mode = Mode.Manual;
    public static double manualVelocity = 0; 
    public static double manualHoodAngle = 25.0; 
    public static double filterGain = 0.6;

    public int shootingState;
    public Robot robot;
    public int cycles;
    boolean shootingSequence = false, holdingSequence = false;
    ElapsedTime timerShoot;
    Pose cameraPose = new Pose(6,6,0);

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new Robot(hardwareMap, Constants.Globals.finalAutoPose, false, Constants.Color.Red, telemetry, gamepad1);
        telemetry.addLine("Init");
        FtcDashboard.getInstance().startCameraStream(robot.camera.ll, 0);
        waitForStart();

        while (opModeIsActive()) {
            
            if (gamepad1.yWasPressed()) {
                robot.setAction(Robot.Actions.Shoot);
            }
            if (gamepad1.yWasReleased()) {
                robot.holdingSequence = false;
            }
            if (gamepad1.leftBumperWasPressed()) robot.outtake.openBlocker();
            if (gamepad1.rightBumperWasPressed()) robot.outtake.closeBlocker();
            if (gamepad1.dpadLeftWasPressed()) robot.outtake.offsetTurretToLeft(2.5);
            if (gamepad1.dpadRightWasPressed()) robot.outtake.offsetTurretToRight(2.5);
            if (gamepad1.psWasPressed()) robot.drive.resetTeleOpHeading();
            if (!robot.shootingSequence) {
                if (gamepad1.right_trigger >= 0.4) {
                    robot.intake.setPower(IntakeMotor.States.Collect);
                } else if (gamepad1.left_trigger >= 0.4) {
                    robot.intake.setPower(IntakeMotor.States.Spit);
                } else {
                    robot.intake.setPower(IntakeMotor.States.Wait);
                }
            }

            if (gamepad1.square) {
                mode = Mode.Physics;
            }
            if (gamepad1.circle) {
                robot.outtake.shooter.updateCoefficients();
            }
            if (gamepad1.optionsWasPressed()) {
               robot.setAction(Robot.Actions.ResetTurretCamera);
            }
            if(gamepad1.shareWasPressed()){
                robot.outtake.shooter.motorDown.pidfVelocity.setFilterGain(filterGain);
            }
            if (mode == Mode.Manual) {
                robot.outtake.shooter.setTargetVelocity(manualVelocity);
                robot.outtake.hood.setAngle(manualHoodAngle);
            } else if (mode == Mode.Physics) {
                robot.outtake.setShooterPhysics();
            }
            robot.update();
            robot.outtake.debug();
            telemetry.addData("Mode", mode);
            telemetry.addData("PID Velo Error", robot.outtake.shooter.motorDown.pidfVelocity.getPositionError());
            telemetry.addData("Robot Pose", robot.drive.getPose());
            telemetry.update();
        }
    }
}