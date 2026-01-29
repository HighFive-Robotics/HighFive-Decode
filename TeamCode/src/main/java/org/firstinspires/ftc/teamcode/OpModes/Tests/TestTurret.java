package org.firstinspires.ftc.teamcode.OpModes.Tests;

import static org.firstinspires.ftc.teamcode.Constants.DeviceNames.turretMotorName;
import static org.firstinspires.ftc.teamcode.Constants.OuttakeConstants.TurretParams.kdTurret;
import static org.firstinspires.ftc.teamcode.Constants.OuttakeConstants.TurretParams.kfTurret;
import static org.firstinspires.ftc.teamcode.Constants.OuttakeConstants.TurretParams.kiTurret;
import static org.firstinspires.ftc.teamcode.Constants.OuttakeConstants.TurretParams.kpTurret;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Core.Hardware.HighMotor;
import org.firstinspires.ftc.teamcode.Core.Module.Outtake.Turret;

@TeleOp
public class TestTurret extends LinearOpMode {

    Turret turret;
    Follower drive;

    @Override
    public void runOpMode() throws InterruptedException {

        turret = new Turret(hardwareMap, Constants.Color.Red);

        drive = Constants.createFollower(hardwareMap);
        drive.setStartingPose(new Pose(8,8, 0));
        drive.startFieldCentricDrive(gamepad1, true, 0);

        waitForStart();

        //+- 1400

        while (opModeIsActive()){
            if(gamepad1.dpadDownWasPressed()){
                turret.motor.resetMotor();
            }

            if(gamepad1.triangleWasPressed()) {
                turret.setTarget(turret.getTargetAngleFromDistance(drive.getPose()));
            }

            if(gamepad1.squareWasPressed()) {
                turret.setTargetDegrees(90);
            }

            if(gamepad1.crossWasPressed()) {
                turret.setTargetDegrees(0);
            }

            if(gamepad1.circleWasPressed()) {
                turret.setTargetDegrees(-90);
            }

            if(gamepad1.dpadLeftWasPressed()) {
                turret.setTargetDegrees(Math.toDegrees(turret.targetAngle)+2);
            }
            if(gamepad1.dpadRightWasPressed()) {
                turret.setTargetDegrees(Math.toDegrees(turret.targetAngle)-2);
            }
            if(gamepad1.dpadUpWasPressed()) {
                turret.reset();
            }

            turret.update();
            drive.update();

            if(gamepad1.psWasPressed()){
                drive.setStartingPose(new Pose(8,8,0));
                drive.setPose(new Pose(8,8,0));
            }
            telemetry.addData("Ticks: ", turret.motor.getCurrentPosition());
            telemetry.addData("Target: ", turret.motor.getTarget());
            telemetry.addData("Power: ", turret.motor.getPower());
            telemetry.addData("targetAngle: ", turret.targetAngle);
            telemetry.addData("current angle : ", turret.currentAngle);
            telemetry.addData("robot pose : ", drive.getPose().toString());
            telemetry.addData("Angle from pose : ", turret.getTargetAngleFromDistance(drive.getPose()));
            telemetry.addData("targetTicks : ", turret.targetTicks);
            telemetry.update();
        }
    }
}
