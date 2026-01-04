package org.firstinspires.ftc.teamcode.OpModes.Tests;

import static org.firstinspires.ftc.teamcode.Constants.Intake.SorterConstants.greenArtifactNumber;
import static org.firstinspires.ftc.teamcode.Constants.Intake.SorterConstants.purpleArtifactNumber;
import static org.firstinspires.ftc.teamcode.Constants.Intake.SorterConstants.sorterColors;
import static org.firstinspires.ftc.teamcode.Core.Module.Intake.Intake.Actions.FindPurple;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Core.Module.Intake.Intake;
import org.firstinspires.ftc.teamcode.Core.Module.Intake.Sorter;

@Disabled
@TeleOp
public class TestSorterColor extends LinearOpMode{

    Intake intake;
    ElapsedTime timer = new ElapsedTime();
    ElapsedTime loopTimer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        intake = new Intake(hardwareMap);
        purpleArtifactNumber = 0;
        greenArtifactNumber = 0;
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

            if(gamepad1.cross && timer.milliseconds() >= 250){
                intake.setAction(FindPurple);
                timer.reset();
            }
            if(gamepad1.square && timer.milliseconds() >= 250){
                intake.setAction(Intake.Actions.FindGreen);
                timer.reset();
            }

            if(gamepad2.cross && timer.milliseconds() >= 250 && intake.sorter.getColor(Sorter.Slots.Slot1) != Constants.Color.Green){
                intake.sorter.setColor(Constants.Color.Green, Sorter.Slots.Slot1);
                greenArtifactNumber++;
                timer.reset();
            }
            if(gamepad2.square && timer.milliseconds() >= 250 && intake.sorter.getColor(Sorter.Slots.Slot1) != Constants.Color.Purple){
                intake.sorter.setColor(Constants.Color.Purple, Sorter.Slots.Slot1);
                purpleArtifactNumber++;
                timer.reset();
            }
            if(gamepad2.circle && timer.milliseconds() >= 250){
                intake.sorter.setColor(Constants.Color.None, Sorter.Slots.Slot1);
                if(intake.sorter.getColor(Sorter.Slots.Slot1) == Constants.Color.Purple){
                    purpleArtifactNumber--;
                }
                if(intake.sorter.getColor(Sorter.Slots.Slot1) == Constants.Color.Green){
                    greenArtifactNumber--;
                }
                timer.reset();
            }

            if(gamepad2.dpad_down && timer.milliseconds() >= 250 && intake.sorter.getColor(Sorter.Slots.Slot2) != Constants.Color.Green){
                intake.sorter.setColor(Constants.Color.Green, Sorter.Slots.Slot2);
                greenArtifactNumber++;
                timer.reset();
            }
            if(gamepad2.dpad_right && timer.milliseconds() >= 250 && intake.sorter.getColor(Sorter.Slots.Slot2) != Constants.Color.Purple){
                intake.sorter.setColor(Constants.Color.Purple, Sorter.Slots.Slot2);
                purpleArtifactNumber++;
                timer.reset();
            }
            if(gamepad2.dpad_left && timer.milliseconds() >= 250){
                intake.sorter.setColor(Constants.Color.None, Sorter.Slots.Slot2);
                if(intake.sorter.getColor(Sorter.Slots.Slot2) == Constants.Color.Purple){
                    purpleArtifactNumber--;
                }
                if(intake.sorter.getColor(Sorter.Slots.Slot2) == Constants.Color.Green){
                    greenArtifactNumber--;
                }
                timer.reset();
            }

            if(gamepad2.right_bumper && timer.milliseconds() >= 250 && intake.sorter.getColor(Sorter.Slots.Slot3) != Constants.Color.Green){
                intake.sorter.setColor(Constants.Color.Green, Sorter.Slots.Slot3);
                greenArtifactNumber++;
                timer.reset();
            }
            if(gamepad2.left_bumper && timer.milliseconds() >= 250 && intake.sorter.getColor(Sorter.Slots.Slot3) != Constants.Color.Purple){
                intake.sorter.setColor(Constants.Color.Purple, Sorter.Slots.Slot3);
                purpleArtifactNumber++;
                timer.reset();
            }
            if(gamepad2.right_trigger >= 0.5 && timer.milliseconds() >= 250){
                intake.sorter.setColor(Constants.Color.None, Sorter.Slots.Slot3);
                if(intake.sorter.getColor(Sorter.Slots.Slot3) == Constants.Color.Purple){
                    purpleArtifactNumber--;
                }
                if(intake.sorter.getColor(Sorter.Slots.Slot3) == Constants.Color.Green){
                    greenArtifactNumber--;
                }
                timer.reset();
            }

            telemetry.addData("Current slot:", intake.sorter.getSlot());
            telemetry.addData("Current number slot:", intake.sorter.slotNumber);
            telemetry.addData("Color 1:", sorterColors[0]);
            telemetry.addData("Color 2:", sorterColors[1]);
            telemetry.addData("Color 3:", sorterColors[2]);
            telemetry.addData("Green:", greenArtifactNumber);
            telemetry.addData("Purple:", purpleArtifactNumber);
            telemetry.addData("Hz", 1.0 / loopTimer.seconds());
            loopTimer.reset();
            telemetry.update();
            intake.update();
        }
    }
}
