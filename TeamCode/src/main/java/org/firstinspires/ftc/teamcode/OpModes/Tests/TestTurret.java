package org.firstinspires.ftc.teamcode.OpModes.Tests;

import static org.firstinspires.ftc.teamcode.Constants.DeviceNames.turretMotorName;
import static org.firstinspires.ftc.teamcode.Constants.OuttakeConstants.TurretParams.kdTurret;
import static org.firstinspires.ftc.teamcode.Constants.OuttakeConstants.TurretParams.kfTurret;
import static org.firstinspires.ftc.teamcode.Constants.OuttakeConstants.TurretParams.kiTurret;
import static org.firstinspires.ftc.teamcode.Constants.OuttakeConstants.TurretParams.kpTurret;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Core.Hardware.HighMotor;

@TeleOp
public class TestTurret extends LinearOpMode {

    HighMotor motor;

    @Override
    public void runOpMode() throws InterruptedException {

        motor = HighMotor.Builder.startBuilding()
                .setMotor(hardwareMap.get(DcMotorEx.class, turretMotorName))
                .setRunMode(HighMotor.RunMode.PID)
                .setReverseMotor(true)
                .setUseZeroPowerBehaviour(false)
                .setPIDCoefficients(kpTurret,kiTurret,kdTurret,kfTurret, HighMotor.FeedForwardType.Lift,1)
                .setEncoder(true,false)
                .build();

        waitForStart();

        //+- 1400

        while (opModeIsActive()){
            if(gamepad1.dpadDownWasPressed()){
                motor.resetMotor();
            }

            if(gamepad1.triangleWasPressed()) {
                motor.setTarget(700);
            }

            if(gamepad1.squareWasPressed()) {
                motor.setTarget(-700);
            }

            if(gamepad1.crossWasPressed()) {
                motor.setTarget(300);
            }

            if(gamepad1.circleWasPressed()) {
                motor.setTarget(0);
            }

            if(gamepad1.dpadRightWasPressed()){
                motor.setPIDCoefficients(kpTurret,kiTurret,kdTurret,kfTurret, HighMotor.FeedForwardType.Lift, 1);
            }
            motor.update();
            telemetry.addData("Ticks: ", motor.getCurrentPosition());
            telemetry.addData("Target: ", motor.getTarget());
            telemetry.addData("Power: ", motor.getPower());
            telemetry.update();
        }
    }
}
