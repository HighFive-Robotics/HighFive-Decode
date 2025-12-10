package org.firstinspires.ftc.teamcode.OpModes.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Core.Module.Intake.Sorter;

@TeleOp
public class TestSorter extends LinearOpMode{

    Sorter sorter;
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {


        sorter = new Sorter(hardwareMap,0);
        waitForStart();
        while(opModeIsActive()){
            if(gamepad1.left_bumper && timer.milliseconds() >= 250){
                sorter.setPreviousSlot();
                timer.reset();
            }
            if(gamepad1.right_bumper && timer.milliseconds() >= 250){
                sorter.setNextSlot();
                timer.reset();
            }

            if(gamepad1.square && timer.milliseconds() >= 250){
                sorter.setSlot(Sorter.Slots.Slot1);
                timer.reset();
            }
            if(gamepad1.cross && timer.milliseconds() >= 250){
                sorter.setSlot(Sorter.Slots.Slot2);
                timer.reset();
            }
            if(gamepad1.circle && timer.milliseconds() >= 250){
                sorter.setSlot(Sorter.Slots.Slot3);
                timer.reset();
            }

            sorter.update();
        }
    }
}
