
package org.firstinspires.ftc.teamcode.Core.Module;


import static org.firstinspires.ftc.teamcode.Constants.DeviceNames.leftBackMotorName;
import static org.firstinspires.ftc.teamcode.Constants.DeviceNames.leftFrontMotorName;
import static org.firstinspires.ftc.teamcode.Constants.DeviceNames.rightBackMotorName;
import static org.firstinspires.ftc.teamcode.Constants.DeviceNames.rightFrontMotorName;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Core.Hardware.HighModule;
import org.firstinspires.ftc.teamcode.Core.Hardware.HighMotor;


public class Drive extends HighModule {

    public HighMotor LFM, RFM, RBM, LBM;
    HardwareMap hardwareMap;
    double driveMultiplier = 1;
    public Drive(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
        LFM = new HighMotor(hardwareMap.get(DcMotorEx.class, leftFrontMotorName), HighMotor.RunMode.Standard, true,true);
        LBM = new HighMotor(hardwareMap.get(DcMotorEx.class, leftBackMotorName), HighMotor.RunMode.Standard,true,true);
        RFM = new HighMotor(hardwareMap.get(DcMotorEx.class, rightFrontMotorName), HighMotor.RunMode.Standard,false,true);
        RBM = new HighMotor(hardwareMap.get(DcMotorEx.class, rightBackMotorName), HighMotor.RunMode.Standard, false, true);

    }

    public void drive(Gamepad gamepad) {
        double forward = -gamepad.left_stick_y * driveMultiplier;
        double strafe = gamepad.left_stick_x * driveMultiplier;
        double turn = gamepad.right_stick_x * driveMultiplier;

        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(turn), 1);

        LFM.setPower((forward + strafe + turn) / denominator);
        LBM.setPower((forward - strafe + turn) / denominator);
        RFM.setPower((forward - strafe - turn) / denominator);
        RBM.setPower((forward + strafe - turn) / denominator);
    }

    public void driveFieldCentric(Gamepad gamepad, double angle) {
        angle = -angle;
        double forward = -gamepad.left_stick_y * driveMultiplier;
        double strafe = gamepad.left_stick_x * driveMultiplier;
        double turn = gamepad.right_stick_x * driveMultiplier;

        double rotX = strafe * Math.cos(angle) - forward * Math.sin(angle);
        double rotY = strafe * Math.sin(angle) + forward * Math.cos(angle);

        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(turn), 1);

        LFM.setPower((rotY + rotX + turn) / denominator);
        LBM.setPower((rotY - rotX + turn) / denominator);
        RFM.setPower((rotY - rotX - turn) / denominator);
        RBM.setPower((rotY + rotX - turn) / denominator);
    }

    @Override
    public void update() {
        LFM.update();
        LBM.update();
        RFM.update();
        RBM.update();
    }
}

