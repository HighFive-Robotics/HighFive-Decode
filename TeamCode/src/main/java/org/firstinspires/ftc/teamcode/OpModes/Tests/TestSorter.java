package org.firstinspires.ftc.teamcode.OpModes.Tests;

import static org.firstinspires.ftc.teamcode.Constants.DeviceNames.intakeMotorName;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Core.Module.Intake.Intake;
import org.firstinspires.ftc.teamcode.Core.Module.Intake.Sorter;

@TeleOp
public class TestSorter extends LinearOpMode{

    Intake intake;
    ElapsedTime timer = new ElapsedTime();
    ElapsedTime loopTimer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

       intake = new Intake(hardwareMap);
        waitForStart();
        while(opModeIsActive()){
            if(gamepad1.left_bumper && timer.milliseconds() >= 250){
                intake.sorter.setPreviousSlot();
                timer.reset();
            }
            if(gamepad1.right_bumper && timer.milliseconds() >= 250){
                intake.sorter.setNextSlot();
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

            telemetry.addData("Current slot:", intake.sorter.getSlot());
            telemetry.addData("Current number slot:", intake.sorter.slotNumber);
            telemetry.addData("Hz", 1.0 / loopTimer.seconds());
            loopTimer.reset();
            telemetry.update();
            intake.update();
        }
    }
}
