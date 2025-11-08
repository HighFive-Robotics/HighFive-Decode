package org.firstinspires.ftc.teamcode.OpModes.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Core.Hardware.HighServo;

@TeleOp
public class ServoTest extends LinearOpMode {

    HighServo s1,s2,s3,s4,s5,s6;
    private enum ServoNames{
        zaza1,
        zaza2,
        zaza3,
        zaza4,
        zaza5,
        zaza6
    }
    int c = 1;
    ServoNames Zaza = ServoNames.zaza1;
    ElapsedTime timer = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        s1 = HighServo.Builder.startBuilding()
                .setServo(hardwareMap.get(Servo.class, "S1"))
                .setStandardRunMode()
                .build();
        s2 = HighServo.Builder.startBuilding()
                .setServo(hardwareMap.get(Servo.class, "S2"))
                .setStandardRunMode()
                .build();
        s3 = HighServo.Builder.startBuilding()
                .setServo(hardwareMap.get(Servo.class, "S3"))
                .setStandardRunMode()
                .build();
        s4 = HighServo.Builder.startBuilding()
                .setServo(hardwareMap.get(Servo.class, "S4"))
                .setStandardRunMode()
                .build();
        s5 = HighServo.Builder.startBuilding()
                .setServo(hardwareMap.get(Servo.class, "S5"))
                .setStandardRunMode()
                .build();
        s6 = HighServo.Builder.startBuilding()
                .setServo(hardwareMap.get(Servo.class, "S6"))
                .setStandardRunMode()
                .build();

        waitForStart();

        while(opModeIsActive()){
            switch (c){
                case 1:
                    Zaza = ServoNames.zaza1;
                    break;
                case 2:
                    Zaza = ServoNames.zaza2;
                    break;
                case 3:
                    Zaza = ServoNames.zaza3;
                    break;
                case 4:
                    Zaza = ServoNames.zaza4;
                    break;
                case 5:
                    Zaza = ServoNames.zaza5;
                    break;
                case 6:
                    Zaza = ServoNames.zaza6;
                    break;
            }
            switch (Zaza){
                case zaza1:
                    if(gamepad1.circle){
                        s1.setPosition(0);
                    }
                    if(gamepad1.cross){
                        s1.setPosition(1);
                    }
                    break;
                case zaza2:
                    if(gamepad1.circle){
                        s2.setPosition(0);
                    }
                    if(gamepad1.cross){
                        s2.setPosition(1);
                    }
                    break;
                case zaza3:
                    if(gamepad1.circle){
                        s3.setPosition(0);
                    }
                    if(gamepad1.cross){
                        s3.setPosition(1);
                    }
                    break;
                case zaza4:
                    if(gamepad1.circle){
                        s4.setPosition(0);
                    }
                    if(gamepad1.cross){
                        s4.setPosition(1);
                    }
                    break;
                case zaza5:
                    if(gamepad1.circle){
                        s5.setPosition(0);
                    }
                    if(gamepad1.cross){
                        s5.setPosition(1);
                    }
                    break;
                case zaza6:
                    if(gamepad1.circle){
                        s6.setPosition(0);
                    }
                    if(gamepad1.cross){
                        s6.setPosition(1);
                    }
                    break;
            }

            if(gamepad1.dpad_down && timer.milliseconds() >= 250){
                if(c == 1){
                    c=6;
                }else c--;
                timer.reset();
            }
            if(gamepad1.dpad_up && timer.milliseconds() >= 250){
                if(c == 6){
                    c=1;
                }else c++;
                timer.reset();
            }


            telemetry.addData("servo", Zaza);
            telemetry.update();
            s1.update();
            s2.update();
            s3.update();
            s4.update();
            s5.update();
            s6.update();
        }
    }
}
