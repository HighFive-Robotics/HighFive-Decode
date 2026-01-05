package org.firstinspires.ftc.teamcode.OpModes.Tests;

import static org.firstinspires.ftc.teamcode.Constants.DeviceNames.intakeMotorName;
import static org.firstinspires.ftc.teamcode.Constants.Intake.SorterConstants.kD;

import static org.firstinspires.ftc.teamcode.Constants.Intake.SorterConstants.kI;
import static org.firstinspires.ftc.teamcode.Constants.Intake.SorterConstants.kP;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Core.Hardware.HighServo;
import org.firstinspires.ftc.teamcode.Core.Module.Intake.Intake;
import org.firstinspires.ftc.teamcode.Core.Module.Intake.Sorter;

@Disabled
@TeleOp
public class PIDSorterTest extends LinearOpMode{

    Intake intake;
    ElapsedTime timer = new ElapsedTime();
    ElapsedTime loopTimer = new ElapsedTime();
    boolean autoCycling = false;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        intake = new Intake(hardwareMap);
        intake.sorter.setSlot(Sorter.Slots.Slot1);
        waitForStart();
        while(opModeIsActive()){


            if(gamepad1.psWasPressed()){
                autoCycling = !autoCycling;
                timer.reset();
            }
            if(gamepad1.square && timer.milliseconds() >= 250){
                intake.sorter.setSlot(Sorter.Slots.Slot1);
                timer.reset();
            }
            if(gamepad1.cross && timer.milliseconds() >= 250){
                intake.sorter.setSlot(Sorter.Slots.Slot2);
                timer.reset();
            }
            if(gamepad1.circle && timer.milliseconds() >= 250){
                intake.sorter.setSlot(Sorter.Slots.Slot3);
                timer.reset();
            }

            if(timer.milliseconds() >= 2500 && autoCycling){
                intake.sorter.setNextSlot();
                timer.reset();
            }
            telemetry.addData("Error:" ,intake.sorter.servo.pidfController.getPositionError()/100);
            telemetry.addData("True Angle:", intake.sorter.servo.getCurrentPositionPID());
            telemetry.addData("Angle:", intake.sorter.servo.getCurrentPositionPID()*100);
            telemetry.addData("Target:", intake.sorter.servo.getTargetPID());
            telemetry.addData("Error:" ,intake.sorter.servo.pidfController.getPositionError());
            telemetry.addData("Auto Cycling:" ,autoCycling);
            telemetry.addData("Current Slot" ,intake.sorter.getSlot());
            telemetry.addData("Hz", 1.0 / loopTimer.seconds());
            loopTimer.reset();
            telemetry.update();
            intake.update();
        }
    }
}
