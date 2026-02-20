package org.firstinspires.ftc.teamcode.Core.Module.Intake;

import static org.firstinspires.ftc.teamcode.Constants.DeviceNames.intakeMotorName;
import static org.firstinspires.ftc.teamcode.Constants.Intake.IntakePowers.powerCollect;
import static org.firstinspires.ftc.teamcode.Constants.Intake.IntakePowers.powerSpit;
import static org.firstinspires.ftc.teamcode.Constants.Intake.IntakePowers.powerTransfer;
import static org.firstinspires.ftc.teamcode.Constants.Intake.IntakePowers.powerWait;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Core.Hardware.HighModule;
import org.firstinspires.ftc.teamcode.Core.Hardware.HighMotor;

@Config
public class IntakeMotor extends HighModule {

    public HighMotor motor;

    public enum States {
        Collect,
        Wait,
        Spit,
        Transfer,
    }

    States state = States.Wait;

    public IntakeMotor(HardwareMap hwMap){
        motor = HighMotor.Builder.startBuilding()
                .setMotor(hwMap.get(DcMotorEx.class, intakeMotorName))
                .setRunMode(HighMotor.RunMode.Standard)
                .setReverseMotor(false)
                .setUseZeroPowerBehaviour(true)
                .build();
    }

    public void setPower(double power) {
        this.power = power;
        motor.setPower(power);
    }

    public void setState(States state){
        this.state = state;
        switch (state){
            case Collect:
                setPower(powerCollect);
                break;
            case Transfer:
                setPower(powerTransfer);
                break;
            case Wait:
                setPower(powerWait);
                break;
            case Spit:
                setPower(powerSpit);
                break;
        }
    }

    public double getPower(){
        return power;
    }

    public States getState() {
        return state;
    }

    @Override
    public void update() {
        motor.update();
    }
}
