package org.firstinspires.ftc.teamcode.OpModes.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Core.Hardware.HighServo;


@TeleOp
public class BuilderTests extends LinearOpMode {
    HighServo servo , servo2, servo3;
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
    }
}
