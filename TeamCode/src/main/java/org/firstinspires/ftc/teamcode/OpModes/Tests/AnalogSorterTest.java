package org.firstinspires.ftc.teamcode.OpModes.Tests;

import static org.firstinspires.ftc.teamcode.Constants.DeviceNames.sorterAnalogInputName;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

@TeleOp
public class AnalogSorterTest extends LinearOpMode {

    AnalogInput analogInput;
    double enc;
    @Override
    public void runOpMode() throws InterruptedException {
        analogInput = hardwareMap.get(analogInput.getClass(), sorterAnalogInputName);

        waitForStart();
        while(opModeIsActive()){
            telemetry.addData("Voltage:", analogInput.getVoltage());
            telemetry.addData("Angle (Based on analog input):", analogInput.getVoltage() / 3.3 * 360);
            telemetry.update();
        }
    }
}



