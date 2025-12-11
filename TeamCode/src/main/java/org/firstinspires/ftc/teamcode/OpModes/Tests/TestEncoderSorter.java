package org.firstinspires.ftc.teamcode.OpModes.Tests;

import static org.firstinspires.ftc.teamcode.Constants.DeviceNames.intakeMotorName;
import static org.firstinspires.ftc.teamcode.Constants.DeviceNames.shooterMotorDownName;
import static org.firstinspires.ftc.teamcode.Constants.DeviceNames.sorterServoName;
import static org.firstinspires.ftc.teamcode.Constants.Intake.SorterConstants.kD;
import static org.firstinspires.ftc.teamcode.Constants.Intake.SorterConstants.kI;
import static org.firstinspires.ftc.teamcode.Constants.Intake.SorterConstants.kP;
import static org.firstinspires.ftc.teamcode.Constants.Intake.SorterConstants.ticksPerRotation;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Core.Hardware.HighEncoder;
import org.firstinspires.ftc.teamcode.Core.Hardware.HighServo;

@Disabled
@TeleOp
public class TestEncoderSorter extends LinearOpMode {

    double power = 0;
    HighServo servo;
    HighEncoder encoder;
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

        waitForStart();
        while(opModeIsActive()){

            if(gamepad1.right_bumper){
                power = 1;
            } else {
                power = 0;
            }
            servo.setPower(power);
            if(gamepad1.cross){
                encoder.resetPosition();
            }
            enc = encoder.getPosition();

            while(enc < 0){
                enc += ticksPerRotation;
            }
            while(enc > ticksPerRotation){
                enc -= ticksPerRotation;
            }
            telemetry.addData("Ticks:", encoder.getPosition());
            telemetry.addData("Pose:", enc);
            telemetry.addData("Angle:", enc / ticksPerRotation * 360);
            telemetry.addData("Velo:", encoder.getVelocity());
            telemetry.addData("Power:", power);
            telemetry.update();
            servo.update();
        }
    }
}
