package org.firstinspires.ftc.teamcode.Core.Module.Intake;

import static org.firstinspires.ftc.teamcode.Constants.Color.Green;
import static org.firstinspires.ftc.teamcode.Constants.Color.None;
import static org.firstinspires.ftc.teamcode.Constants.Color.Purple;
import static org.firstinspires.ftc.teamcode.Constants.DeviceNames.breakBeamIntakeNameUp;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Core.Hardware.HighModule;
import org.firstinspires.ftc.teamcode.Core.Hardware.HighSensor;

public class Intake extends HighModule {
    public IntakeMotor intakeMotor;
    public HighSensor sensor;
    public DigitalChannel breakBeam;

    public Constants.Color currentColor;
    public int currentSlotNumber;
    public long count = 0;

    ElapsedTime timer = new ElapsedTime(), intakeHelper = new ElapsedTime();

    public boolean artifactPassThrough = false;
    public boolean breakBeamCollected = false, helpingSorter = false;
    private boolean colorAssignedToCurrentSample = false;
    public boolean canStop = true;

    States state = States.Collect;
    IntakeMotor.States lastPower = IntakeMotor.States.Wait;
    CollectTypes collectType = CollectTypes.Sorted;
    Actions currentAction;
    public enum States {
        Collect,
        Wait,
    }

    public enum Actions {
        FindGreen,
        FindPurple,
        NextSlot,
        PreviousSlot
    }

    public enum CollectTypes {
        Sorted,
        Normal,
        Mix
    }

    public Intake(HardwareMap hwMap) {
        intakeMotor = new IntakeMotor(hwMap);
        breakBeam = hwMap.get(DigitalChannel.class, breakBeamIntakeNameUp);
        breakBeam.setMode(DigitalChannel.Mode.INPUT);
    }


    public void setState(States state) {
        this.state = state;
    }

    public void setAction(Actions action) {
        this.currentAction = action;
        switch (action) {
            case FindGreen: {
            }
            break;
        }
    }

    @Override
    public void update() {
        intakeMotor.update();
    }
}