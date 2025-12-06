package org.firstinspires.ftc.teamcode.OpModes.Tests;

import static org.firstinspires.ftc.teamcode.Constants.DeviceNames.intakeMotorName;
import static org.firstinspires.ftc.teamcode.Constants.DeviceNames.shooterMotorDownName;
import static org.firstinspires.ftc.teamcode.Constants.DeviceNames.sorterServoName;
import static org.firstinspires.ftc.teamcode.Constants.Intake.SorterConstants.kD;
import static org.firstinspires.ftc.teamcode.Constants.Intake.SorterConstants.kI;
import static org.firstinspires.ftc.teamcode.Constants.Intake.SorterConstants.kP;
import static org.firstinspires.ftc.teamcode.Constants.Intake.SorterConstants.ticksPerRotation;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Core.Hardware.HighEncoder;
import org.firstinspires.ftc.teamcode.Core.Hardware.HighServo;

@Config
@TeleOp
public class PIDSorterTest extends LinearOpMode {

    double power = 0;
    HighServo servo;
    HighEncoder encoder;
    public static double target=0;
    double enc;
    @Override
    public void runOpMode() throws InterruptedException {
        encoder = new HighEncoder(hardwareMap.get(DcMotorEx.class, intakeMotorName), 0, true);

        servo = HighServo.Builder.startBuilding()
                .setServo(hardwareMap.get(CRServo.class , sorterServoName))
                .setPIDRunMode()
                .setPIDCoefficients(kP,kI,kD,0)
                .setEncoderResolution(ticksPerRotation)
                .setEncoder(encoder)
                .build();
        servo.pidfController.setTelemetry(telemetry);
        waitForStart();
        while(opModeIsActive()){
            servo.setPIDCoefficients(kP,kI,kD);
            if(gamepad1.cross){
                target = 270;
            }
            if(gamepad1.square){
                target=90;
            }
            if(gamepad1.circle){
                target=0;
            }
            servo.setTarget(target);
            telemetry.addData("Angle:", servo.getCurrentPositionPID());
            telemetry.addData("Error:" , servo.pidfController.getPositionError());
            telemetry.addData("Target:" , target);
            telemetry.addData("Power:", servo.getPowerPID(servo.getCurrentPositionPID()));
            telemetry.update();
            servo.update();
        }
    }
}
