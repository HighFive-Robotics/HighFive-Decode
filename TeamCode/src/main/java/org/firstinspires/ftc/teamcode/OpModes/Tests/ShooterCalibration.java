package org.firstinspires.ftc.teamcode.OpModes.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Core.Module.Others.Drive;

@TeleOp(name = "Overall Tests")
public class ShooterCalibration extends LinearOpMode {
    DcMotorEx mU , mD;
    public static double target=5;
    public static double kp=0.001,ki=0.002,kd=0,kf=0.00016;
    Telemetry graph;
    Drive teleopDrive;
    ElapsedTime timer;
    FtcDashboard dashboard;


    @Override
    public void runOpMode() throws InterruptedException {
        mU = hardwareMap.get(DcMotorEx.class , "OMT");
        mD = hardwareMap.get(DcMotorEx.class , "OMB");
        mD.setDirection(DcMotorSimple.Direction.REVERSE);
        mU.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mU.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        timer = new ElapsedTime();
        dashboard = FtcDashboard.getInstance();
        teleopDrive = new Drive(hardwareMap);
        boolean pressA = false;
        boolean pressB = false;
        boolean pressX = false;
        waitForStart();
        while(opModeIsActive()){

            teleopDrive.drive(gamepad1);
            if(gamepad1.a && timer.milliseconds()>=250){
                if(!pressA){
                    mU.setPower(0.76);
                }else {
                    mU.setPower(0);
                }
                timer.reset();
                pressA = !pressA;
            }
            if(gamepad1.b && timer.milliseconds()>=250){
                if(!pressB){
                    mD.setPower(0.76);
                }else {
                    mD.setPower(0);
                }
                timer.reset();
                pressB = !pressB;
            }
            if(gamepad1.x && timer.milliseconds() >= 250){
                if(!pressX){
                    mD.setPower(1);
                    mU.setPower(1);
                }else {
                    mD.setPower(0);
                    mU.setPower(0);
                }
                timer.reset();
                pressX = !pressX;
            }
            telemetry.addData("OMT power" , mU.getPower());
            telemetry.addData("OMB power" , mD.getPower());
            telemetry.addData("LBM power" , teleopDrive.LBM.getPower());
            telemetry.addData("LFM power" , teleopDrive.LFM.getPower());
            telemetry.addData("RBM power" , teleopDrive.RBM.getPower());
            telemetry.addData("RFM power" , teleopDrive.RFM.getPower());
            //Current Possitions
            telemetry.addData("OMT pos" ,-1*mU.getCurrentPosition());
            telemetry.addData("OMT velobelo" ,-1*mU.getVelocity());
            telemetry.addData("OMT velobelo" ,-1*(mU.getVelocity()/28)*0.072);
            telemetry.addData("OMB pos" , mD.getCurrentPosition());
            telemetry.addData("LBM pos" , teleopDrive.LBM.getCurrentPosition());
            telemetry.addData("LFM pos" , teleopDrive.LFM.getCurrentPosition());
            telemetry.addData("RBM pos" , teleopDrive.RBM.getCurrentPosition());
            telemetry.addData("RFM pos" , teleopDrive.RFM.getCurrentPosition());

            telemetry.update();
        }
    }
}
