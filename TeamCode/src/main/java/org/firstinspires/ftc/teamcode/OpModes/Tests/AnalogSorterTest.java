package org.firstinspires.ftc.teamcode.OpModes.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Core.Module.Others.Led;

@TeleOp
public class AnalogSorterTest extends LinearOpMode {

    //AnalogInput analogInput;
    Led led;
    double enc;
    @Override
    public void runOpMode() throws InterruptedException {
       // analogInput = hardwareMap.get(analogInput.getClass(), sorterAnalogInputName);
        led = new Led(hardwareMap);

        waitForStart();
        while(opModeIsActive()){
          //  telemetry.addData("Voltage:", analogInput.getVoltage());
           // telemetry.addData("Angle (Based on analog input):", analogInput.getVoltage() / 3.3 * 360);
            telemetry.update();
            if(gamepad1.crossWasPressed()){
                led.setColor(Constants.Color.Green);
            }
            if(gamepad1.squareWasPressed()){
                led.setColor(Constants.Color.Purple);
            }
            if(gamepad1.circleWasPressed()){
                led.setColor(Constants.Color.Red);
            }
            led.update();
        }
    }
}



