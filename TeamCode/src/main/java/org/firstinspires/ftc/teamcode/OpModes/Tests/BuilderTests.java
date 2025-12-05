package org.firstinspires.ftc.teamcode.OpModes.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Core.Hardware.HighMotor;
import org.firstinspires.ftc.teamcode.Core.Hardware.HighServo;

@Disabled
@TeleOp
public class BuilderTests extends LinearOpMode {
    HighServo servo , servo2, servo3;
    HighMotor motor;
    AnalogInput analogInput;
    @Override
    public void runOpMode() throws InterruptedException {
        servo = HighServo.Builder.startBuilding()
                .setServo(hardwareMap.get(Servo.class , "example"))
                .setStandardRunMode()
                .setAnalogInput(analogInput)
                .setAnalogInputCoefficients(0,0,0,0,0)
                .setInitPosition(0,true)
                .build();
        servo2 = HighServo.Builder.startBuilding()
                .setServo(hardwareMap.get(Servo.class , "example"))
                .setContinousRotationRunMode()
                .build();
        servo3 = HighServo.Builder.startBuilding()
                .setServo(hardwareMap.get(Servo.class , "example"))
                .setMotionProfilerRunMode()
                .setMotionProfilerCoefficients(0,0,0)
                .build();
        servo = HighServo.Builder.startBuilding()
                .setServo(hardwareMap.get(Servo.class , "example"))
                .setStandardRunMode()
                .setAnalogInput(analogInput)
                .setAnalogInputCoefficients(0,0,0,0,0)
//                .setInitPosition(0,true)  OPTIONALA
                .build();
        servo = HighServo.Builder.startBuilding()
                .setServo(hardwareMap.get(CRServo.class , "example"))
                .setContinousRotationRunMode()
                .setAnalogInput(analogInput)
                .setAnalogInputCoefficients(0,0,0,0,0)
                .build();
        motor = HighMotor.Builder.startBuilding()
                .setMotor(hardwareMap.get(DcMotorEx.class , "example"))
                .setRunMode(HighMotor.RunMode.Standard)
                .setReverseMotor(false)
                .build();
        motor = HighMotor.Builder.startBuilding()
                .setMotor(hardwareMap.get(DcMotorEx.class , "example"))
                .setRunMode(HighMotor.RunMode.PID)
                .setReverseMotor(false)
                .setUseZeroPowerBehaviour(true)
                .setEncoder(true , false)
                .setPIDCoefficients(0,0,0,0)
                .build();
        motor = HighMotor.Builder.startBuilding()
                .setMotor(hardwareMap.get(DcMotorEx.class , "example"))
                .setRunMode(HighMotor.RunMode.Squid)
                .setReverseMotor(false)
                .setUseZeroPowerBehaviour(true)
                .setMultiplier(1)
                .setEncoder(true , false)
                .setSquidCoefficients(0,0,0,0)
                .build();
        motor = HighMotor.Builder.startBuilding()
                .setMotor(hardwareMap.get(DcMotorEx.class , "example"))
                .setRunMode(HighMotor.RunMode.Velocity)
                .setReverseMotor(false)
                .setUseZeroPowerBehaviour(true)
                .setMultiplier(1)
                .setEncoder(true , false)
                .setVelocityPIDCoefficients(0,0,0,0,1)
                .build();

    }
}
