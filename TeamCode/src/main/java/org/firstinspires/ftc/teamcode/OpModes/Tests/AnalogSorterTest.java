package org.firstinspires.ftc.teamcode.OpModes.Tests;

import static org.firstinspires.ftc.teamcode.Constants.DeviceNames.intakeMotorName;
import static org.firstinspires.ftc.teamcode.Constants.DeviceNames.shooterMotorDownName;
import static org.firstinspires.ftc.teamcode.Constants.DeviceNames.sorterAnalogInputName;
import static org.firstinspires.ftc.teamcode.Constants.DeviceNames.sorterServoName;
import static org.firstinspires.ftc.teamcode.Constants.Intake.SorterConstants.kD;
import static org.firstinspires.ftc.teamcode.Constants.Intake.SorterConstants.kI;
import static org.firstinspires.ftc.teamcode.Constants.Intake.SorterConstants.kP;
import static org.firstinspires.ftc.teamcode.Constants.Intake.SorterConstants.ticksPerRotation;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Core.Hardware.HighEncoder;
import org.firstinspires.ftc.teamcode.Core.Hardware.HighServo;

@Disabled
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



