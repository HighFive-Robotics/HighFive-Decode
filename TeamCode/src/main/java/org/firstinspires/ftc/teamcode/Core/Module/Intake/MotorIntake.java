package org.firstinspires.ftc.teamcode.Core.Module.Intake;

import static org.firstinspires.ftc.teamcode.Constants.DeviceNames.intakeMotorName;
import static org.firstinspires.ftc.teamcode.Constants.DeviceNames.rightBackMotorName;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Core.Hardware.HighModule;
import org.firstinspires.ftc.teamcode.Core.Hardware.HighMotor;

@Config
public class MotorIntake extends HighModule {

    public static double powerWait = 0, powerCollect = 1, powerSpit = -1, powerTransfer = 0.5;
    HighMotor motor, inactiveMotor;

    public enum States {
        Collect,
        Wait,
        Spit,
        Transfer
    }

    States state = States.Wait;

    public MotorIntake(HardwareMap hwMap){
        motor = new HighMotor(hwMap.get(DcMotorEx.class, intakeMotorName), HighMotor.RunMode.Standard, true, true);

        /*motor = HighMotor.Builder.startBuilding()
                .setMotor(hwMap.get(DcMotorEx.class, intakeMotorName))
                .setRunMode(HighMotor.RunMode.Standard)
                .setReverseMotor(true)
                .build();*/
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

    public States getState() {
        return state;
    }

    @Override
    public void update() {
        motor.update();
    }
    public void disable(){
        setState(States.Wait);
        inactiveMotor = motor;
        motor = null;
    }
    public void enable(){
        motor = inactiveMotor;
    }
}
