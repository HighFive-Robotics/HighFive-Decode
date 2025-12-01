package org.firstinspires.ftc.teamcode.OpModes.Tests;

import static org.firstinspires.ftc.teamcode.Constants.DeviceNames.intakeSensorName;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Core.Hardware.HighSensor;

@TeleOp
public class SensorTest extends LinearOpMode {

    HighSensor sensor;
    @Override
    public void runOpMode() throws InterruptedException {
        sensor = new HighSensor(hardwareMap, intakeSensorName);

        waitForStart();
        while(opModeIsActive()){
            sensor.update();
            sensor.telemetry(telemetry);
            telemetry.update();
        }
    }
}
