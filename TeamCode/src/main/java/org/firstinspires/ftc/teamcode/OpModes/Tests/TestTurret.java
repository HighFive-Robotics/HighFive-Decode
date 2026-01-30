package org.firstinspires.ftc.teamcode.OpModes.Tests;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Core.Module.Outtake.Turret;

@TeleOp
public class TestTurret extends LinearOpMode {

    ElapsedTime time = new ElapsedTime();
    Turret turret;
    Follower drive;

    @Override
    public void runOpMode() throws InterruptedException {
        turret = new Turret(hardwareMap, Constants.Color.Red);

        drive = Constants.createFollower(hardwareMap);
        drive.setStartingPose(new Pose(8,8, 0));
        drive.startFieldCentricDrive(gamepad1, true, 0);

        waitForStart();

        time.reset();

        //+- 1400

        while (opModeIsActive()){
            if(gamepad1.dpadDownWasPressed()){
                turret.motor.resetMotor();
            }

            if(gamepad1.triangle && time.milliseconds() >= 50) {
                turret.setTarget(turret.getTargetAngleFromDistance(drive.getPose()));
            }

            if(gamepad1.squareWasPressed()) {
                turret.setTargetDegrees(90);
            }

            if(gamepad1.crossWasPressed()) {
                turret.setTargetTicks(0);
            }

            if(gamepad1.circleWasPressed()) {
                turret.setTargetDegrees(-90);
            }

            if(gamepad1.dpadLeftWasPressed()) {
                turret.setTarget(turret.getTarget()+0.025);
            }
            if(gamepad1.dpadRightWasPressed()) {
                turret.setTarget(turret.getTarget()-0.025);
            }

            if(gamepad1.leftBumperWasPressed()) {
                turret.set
            }
            if(gamepad1.rightBumperWasPressed()) {
                turret.setTarget(turret.getTarget()-0.025);
            }

            if(gamepad1.optionsWasPressed()) {
                turret.reset();
            }

            turret.update();
            drive.update();

            if(gamepad1.psWasPressed()){
                drive.setStartingPose(new Pose(6,6,0));
                drive.setPose(new Pose(6,6,0));
            }
            telemetry.addData("Ticks: ", turret.getCurrentTicks());
            telemetry.addData("Target Ticks: ", turret.getTargetTicks());
            telemetry.addData("Power: ", turret.motor.getPower());
            telemetry.addData("Current angle: ", turret.getCurrentAngle());
            telemetry.addData("Current angle degrees: ", turret.getCurrentAngleDegrees());
            telemetry.addData("Target angle: ", turret.getTarget());
            telemetry.addData("Target angle degrees ", turret.getTargetDegrees());
            telemetry.addData("robot pose : ", drive.getPose().toString());
            telemetry.addData("Angle from pose : ", turret.getTargetAngleFromDistance(drive.getPose()));
            telemetry.update();
        }
    }
}
