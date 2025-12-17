package org.firstinspires.ftc.teamcode.Core.Module.Intake;

import static org.firstinspires.ftc.teamcode.Constants.DeviceNames.sorterServoName;
import static org.firstinspires.ftc.teamcode.Constants.Intake.SorterConstants.kD;
import static org.firstinspires.ftc.teamcode.Constants.Intake.SorterConstants.kF;
import static org.firstinspires.ftc.teamcode.Constants.Intake.SorterConstants.kI;
import static org.firstinspires.ftc.teamcode.Constants.Intake.SorterConstants.kP;
import static org.firstinspires.ftc.teamcode.Constants.Intake.SorterConstants.sorterColors;
import static org.firstinspires.ftc.teamcode.Constants.Intake.SorterConstants.targetSlot1;
import static org.firstinspires.ftc.teamcode.Constants.Intake.SorterConstants.targetSlot2;
import static org.firstinspires.ftc.teamcode.Constants.Intake.SorterConstants.targetSlot3;
import static org.firstinspires.ftc.teamcode.Constants.Intake.SorterConstants.ticksPerRotation;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Core.Hardware.HighEncoder;
import org.firstinspires.ftc.teamcode.Core.Hardware.HighModule;
import org.firstinspires.ftc.teamcode.Core.Hardware.HighServo;

@Config
public class Sorter extends HighModule {
    public HighServo servo;
    public HighEncoder encoder;
    public int slotNumber = 1;
    private final double tolerance;
    public boolean isFull = false;

    public enum States {
        Manual,
        Automated
    }

    public enum Slots {
        Slot1,
        Slot2,
        Slot3
    }

    Slots slot = Slots.Slot2;
    States state = States.Automated;

    public Sorter(HardwareMap hwMap, DcMotorEx motor, double offset) {
        encoder = new HighEncoder(motor, offset, false);
        servo = HighServo.Builder.startBuilding()
                .setServo(hwMap.get(CRServo.class, sorterServoName))
                .setPIDRunMode()
                .setPIDCoefficients(kP, kI, kD, kF)
                .setEncoderResolution(ticksPerRotation)
                .setEncoder(encoder)
                .build();

        tolerance = 1.1;
        servo.pidfController.setTolerance(tolerance);
        setSlot(Slots.Slot2);
        slotNumber = 2;
        /// TODO DO NOT MAKE NONE
        sorterColors[0] = Constants.Color.None;
        sorterColors[1] = Constants.Color.None;
        sorterColors[2] = Constants.Color.None;
    }

    public void setSlot(Slots slot) {
        this.slot = slot;
        switch (slot) {
            case Slot1: {
                setTarget(targetSlot1);
                slotNumber = 1;
            }
            break;
            case Slot2: {
                setTarget(targetSlot2);
                slotNumber = 2;
            }
            break;
            case Slot3: {
                setTarget(targetSlot3);
                slotNumber = 3;
            }
            break;
        }
    }

    public void setSlot(int slotNumber) {
        slotNumber = Range.clip(slotNumber, 1, 3);
        this.slotNumber = slotNumber;
        switch (slotNumber) {
            case 1: {
                setTarget(targetSlot1);
                slot = Slots.Slot1;
            }
            break;
            case 2: {
                setTarget(targetSlot2);
                slot = Slots.Slot2;
            }
            break;
            case 3: {
                setTarget(targetSlot3);
                slot = Slots.Slot3;
            }
            break;
        }
    }

    public int getNextSlot() {
        switch (slotNumber) {
            case 1: {
                return 2;
            }
            case 2: {
                return 3;
            }
            default: {
                return 1;
            }
        }
        //return 0;
    }

    public void setNextSlot() {
        switch (slotNumber) {
            case 1: {
                slotNumber = 2;
                setTarget(targetSlot2);
                slot = Slots.Slot2;
            }
            break;
            case 2: {
                slotNumber = 3;
                setTarget(targetSlot3);
                slot = Slots.Slot3;
            }
            break;
            case 3: {
                slotNumber = 1;
                setTarget(targetSlot1);
                slot = Slots.Slot1;
            }
            break;
        }
    }

    public void setPreviousSlot() {
        switch (slotNumber) {
            case 1: {
                slotNumber = 3;
                setTarget(targetSlot3);
                slot = Slots.Slot3;
            }
            break;
            case 2: {
                slotNumber = 1;
                setTarget(targetSlot1);
                slot = Slots.Slot1;
            }
            break;
            case 3: {
                slotNumber = 2;
                setTarget(targetSlot2);
                slot = Slots.Slot2;
            }
            break;
        }
    }

    public Slots getSlot() {
        return slot;
    }

    public Constants.Color getColor(Slots slot) {
        switch (slot) {
            case Slot1: {
                return sorterColors[0];
            }
            case Slot2: {
                return sorterColors[1];
            }
            case Slot3: {
                return sorterColors[2];
            }
        }
        return Constants.Color.None;
    }

    public Constants.Color getColor(int slotNumber) {
        switch (slotNumber) {
            case 1: {
                return sorterColors[0];
            }
            case 2: {
                return sorterColors[1];
            }
            case 3: {
                return sorterColors[2];
            }
        }
        return Constants.Color.None;
    }

    public int getSlotNumber() {
        return slotNumber;
    }

    public void setColor(Constants.Color color, Slots slot) {
        switch (slot) {
            case Slot1: {
                sorterColors[0] = color;
            }
            break;
            case Slot2: {
                sorterColors[1] = color;
            }
            break;
            case Slot3: {
                sorterColors[2] = color;
            }
            break;
        }
    }

    public void setState(States state) {
        this.state = state;
    }

    public States getState() {
        return state;
    }

    @Override
    public void setTarget(double target) {
        this.target = target;
        servo.setTarget(target);
    }
    @Override
    public double getTarget() {
        return target;
    }
    @Override
    public boolean atTarget() {
        return servo.pidfController.getPositionError() <= tolerance;
    }
    @Override
    public void update() {
        isFull = sorterColors[0] != Constants.Color.None && sorterColors[1] != Constants.Color.None && sorterColors[2] != Constants.Color.None;
        servo.update();
    }
}
