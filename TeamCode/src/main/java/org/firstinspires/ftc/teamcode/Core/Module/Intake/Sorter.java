package org.firstinspires.ftc.teamcode.Core.Module.Intake;

import static org.firstinspires.ftc.teamcode.Constants.DeviceNames.shooterMotorDownName;
import static org.firstinspires.ftc.teamcode.Constants.DeviceNames.sorterServoName;
import static org.firstinspires.ftc.teamcode.Constants.Intake.SorterConstants.sorterColors;
import static org.firstinspires.ftc.teamcode.Constants.Intake.SorterConstants.targetSlot1;
import static org.firstinspires.ftc.teamcode.Constants.Intake.SorterConstants.targetSlot2;
import static org.firstinspires.ftc.teamcode.Constants.Intake.SorterConstants.targetSlot3;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Core.Hardware.HighEncoder;
import org.firstinspires.ftc.teamcode.Core.Hardware.HighModule;
import org.firstinspires.ftc.teamcode.Core.Hardware.HighServo;

@Config
public class Sorter extends HighModule {
    HighServo servo;
    HighEncoder encoder;
    public int slotNumber = 1;
    private final double tolerance;
    boolean isFull = false;

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

    public Sorter(HardwareMap hwMap) {
        servo = HighServo.Builder.startBuilding()
                .setServo(hwMap.get(CRServo.class , sorterServoName))
                .setContinousRotationRunMode()
                .build();
        encoder = new HighEncoder(hwMap.get(DcMotorEx.class, shooterMotorDownName), 0, false);
        encoder.resetPosition();
        servo.setTolerance(2);
        tolerance = servo.getTolerance();
        setSlot(Slots.Slot2);
    }

    public void setSlot(Slots slot) {
        this.slot = slot;
        switch (slot){
            case Slot1: {
                servo.setTarget(targetSlot1);
                slotNumber = 1;
            }
            break;
            case Slot2: {
                servo.setTarget(targetSlot2);
                slotNumber = 2;
            }
            break;
            case Slot3: {
                servo.setTarget(targetSlot3);
                slotNumber = 3;
            }
            break;
        }
    }

    public void setSlot(int slotNumber) {
        slotNumber = Range.clip(slotNumber, 1, 3);
        this.slotNumber = slotNumber;
        switch (slotNumber){
            case 1: {
                servo.setTarget(targetSlot1);
            }
            break;
            case 2: {
                servo.setTarget(targetSlot2);
            }
            break;
            case 3: {
                servo.setTarget(targetSlot3);
            }
            break;
        }
    }

    public void setNextSlot(){
        switch (slotNumber){
            case 1:{
                slotNumber = 2;
                servo.setTarget(targetSlot2);
            }
            case 2:{
                slotNumber = 3;
                servo.setTarget(targetSlot3);
            }
            case 3:{
                slotNumber = 1;
                servo.setTarget(targetSlot1);
            }
        }
    }

    public void setPreviousSlot(){
        switch (slotNumber){
            case 1:{
                slotNumber = 3;
                servo.setTarget(targetSlot3);
            }
            case 2:{
                slotNumber = 1;
                servo.setTarget(targetSlot1);
            }
            case 3:{
                slotNumber = 2;
                servo.setTarget(targetSlot2);
            }
        }
    }

    public Slots getSlot() {
        return slot;
    }

    public Constants.Color getColor(Slots slot) {
        switch (slot){
            case Slot1:{
                return sorterColors[0];
            }
            case Slot2:{
                return sorterColors[1];
            }
            case Slot3:{
                return sorterColors[2];
            }
        }
        return Constants.Color.None;
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
    public boolean atTarget(){
        return servo.pidfController.getPositionError() <= tolerance;
    }

    @Override
    public void update() {
        isFull = sorterColors[0] != Constants.Color.None && sorterColors[1] != Constants.Color.None && sorterColors[2] != Constants.Color.None;
        servo.update();
    }
}
