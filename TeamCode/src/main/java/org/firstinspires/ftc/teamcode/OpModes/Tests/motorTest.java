package org.firstinspires.ftc.teamcode.OpModes.Tests;

import static org.firstinspires.ftc.teamcode.Constants.DeviceNames.leftBackMotorName;
import static org.firstinspires.ftc.teamcode.Constants.DeviceNames.leftFrontMotorName;
import static org.firstinspires.ftc.teamcode.Constants.DeviceNames.rightBackMotorName;
import static org.firstinspires.ftc.teamcode.Constants.DeviceNames.rightFrontMotorName;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Am ajuns rau")
public class motorTest  extends LinearOpMode {
    DcMotorEx lfm;
    DcMotorEx rfm;
    DcMotorEx rbm;
    DcMotorEx lbm;
    @Override
    public void runOpMode() throws InterruptedException {
        lfm = hardwareMap.get(DcMotorEx.class, leftFrontMotorName);
        rfm = hardwareMap.get(DcMotorEx.class, rightFrontMotorName);
        lbm = hardwareMap.get(DcMotorEx.class, leftBackMotorName);
        rbm = hardwareMap.get(DcMotorEx.class, rightBackMotorName);

        waitForStart();
        while (opModeIsActive()){
            if(gamepad1.a){
                rfm.setPower(1); //LBM
            }else rfm.setPower(0);
            if(gamepad1.b){
                lfm.setPower(1);
            }else lfm.setPower(0);
            if(gamepad1.x){
                rbm.setPower(1); //RFM
            }else rbm.setPower(0);
            if(gamepad1.y){
                lbm.setPower(1);
            }else lbm.setPower(0);//RBM
        }
    }
}
