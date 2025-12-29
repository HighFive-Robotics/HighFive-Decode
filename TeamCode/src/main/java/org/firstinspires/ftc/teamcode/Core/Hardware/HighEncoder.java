package org.firstinspires.ftc.teamcode.Core.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class HighEncoder {

    public DcMotorEx encoder;
    public int reverseMultiplier = 1;
    public double offset;

    public HighEncoder(DcMotorEx encoder, double offset, boolean reversed)
    {
        this.encoder = encoder;
        this.offset = offset;
        if(reversed){
            reverseMultiplier = -1;
        }
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void resetPosition()
    {
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        offset = 0;
    }
    public double getPosition()
    {
        return offset + encoder.getCurrentPosition() * reverseMultiplier;
    }

    public double getVelocity(){
        return encoder.getVelocity() * reverseMultiplier;
    }
    public double getVelocityDeg(){
        return encoder.getVelocity(AngleUnit.DEGREES) * reverseMultiplier;
    }
}
