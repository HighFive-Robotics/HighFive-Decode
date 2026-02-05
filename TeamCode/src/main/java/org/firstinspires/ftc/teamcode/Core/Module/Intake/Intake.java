package org.firstinspires.ftc.teamcode.Core.Module.Intake;

import static org.firstinspires.ftc.teamcode.Constants.DeviceNames.breakBeamIntakeNameDown;
import static org.firstinspires.ftc.teamcode.Constants.DeviceNames.breakBeamIntakeNameUp;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Core.Hardware.HighModule;

public class Intake extends HighModule {
    
    public IntakeMotor intakeMotor;
    public DigitalChannel breakBeamDown, breakBeamUp;

    ElapsedTime timer = new ElapsedTime();

    public boolean isFull = false, isPartial = false;

    public Intake(HardwareMap hwMap) {
        intakeMotor = new IntakeMotor(hwMap);
        breakBeamUp = hwMap.get(DigitalChannel.class, breakBeamIntakeNameUp);
        breakBeamUp.setMode(DigitalChannel.Mode.INPUT);
        breakBeamDown = hwMap.get(DigitalChannel.class, breakBeamIntakeNameDown);
        breakBeamDown.setMode(DigitalChannel.Mode.INPUT);
    }

    public void setPowerState(IntakeMotor.States state){
        intakeMotor.setState(state);
    }

    public void setPower(double power){
        intakeMotor.setPower(power);
    }

    @Override
    public void update() {
        intakeMotor.update();
        if(timer.milliseconds() >= 150){
            isFull = breakBeamUp.getState() && breakBeamDown.getState();
            isPartial = breakBeamUp.getState();
            timer.reset();
        }
    }
}