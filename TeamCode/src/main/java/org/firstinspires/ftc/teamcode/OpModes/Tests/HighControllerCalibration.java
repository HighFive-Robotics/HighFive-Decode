package org.firstinspires.ftc.teamcode.OpModes.Tests;


import static org.firstinspires.ftc.teamcode.Constants.DeviceNames.intakeMotorName;
import static org.firstinspires.ftc.teamcode.Constants.DeviceNames.sorterServoName;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Core.Algorithms.HighController;
import org.firstinspires.ftc.teamcode.Core.Hardware.HighEncoder;

@TeleOp(name = "HighController Tuning")
@Config
public class HighControllerCalibration extends LinearOpMode {
    public static double kP = 0.00055,kD=0.00006,kI=0,
            staticGain=0.068,dynamicGain=0,inertialGain=0,
            bangBangZone=100, precisionZone= 10 ,slewRate=5,
            tolerance =2 , lookAhead= 0.05 , breakingGain = 2.0,
            spinOpposingGain=1.1 , spinOpposingPrefix=-1.0,
            bangBangGain = 0.92 , additionMin = 0.5 , additionMax=1,
            viscousGain = 2.5;
    public static boolean useOpposingCorrection = true;
    public static boolean useActualVelocity = true;
    private double target;
    private double tpr = 8192;
    CRServo servo;
    HighController controller;
    HighController.Kernel kernel;
    HighEncoder encoder;

    @Override
    public void runOpMode() throws InterruptedException {
        servo = hardwareMap.get(CRServo.class,sorterServoName);
        kernel = new HighController.Kernel.Builder()
                .setPID(kP,kI,kD)
                .setFeedforward(staticGain,dynamicGain,inertialGain)
                .setBangBangRange(bangBangZone)
                .setTolerance(tolerance)
                .setPrecisionThreshold(precisionZone)
                .setSlewRate(slewRate)
                .setOscillationDampener(lookAhead,breakingGain)
                .setOpposingCorrection(useOpposingCorrection,spinOpposingGain,spinOpposingPrefix)
                .setBangBangGain(bangBangGain)
                .setPrecisionAddition(additionMin,additionMax)
                .build();
        encoder = new HighEncoder(hardwareMap.get(DcMotorEx.class,intakeMotorName),0,true);
        controller = new HighController(kernel, HighController.Modes.Angular);
        boolean autoCycling=false;
        ElapsedTime autoTimer = new ElapsedTime();
        waitForStart();
        while(opModeIsActive()) {
            double angle = handleAngle();
            double actualVelo = encoder.getVelocityDeg();
            if(gamepad1.psWasPressed()){
                autoCycling = !autoCycling;
                target = 0;
                autoTimer.reset();
            }
            if(gamepad1.crossWasPressed()){
                target = 0;
            }
            if(gamepad1.circleWasPressed()){
                target = 120;
            }
            if(gamepad1.squareWasPressed()){
                target = 240;
            }
            if(gamepad1.triangleWasPressed()){
                target = 90;
            }
            if(autoCycling){
                double step = 80;
                if(autoTimer.milliseconds() >= 3000){
                    if(target + step > 360){
                        target = 0;
                    }else target += step;
                    autoTimer.reset();
                }
            }
            updateKernel();
            controller.setTarget(target);
            double power;
            if(useActualVelocity){
                 power = controller.run(angle,actualVelo);
            }else  power = controller.run(angle);
            servo.setPower(power);
            telemetry.addData("Auto Cycle" , autoCycling);
            telemetry.addData("Power" , power);
            telemetry.addData("Current Position" , angle);
            telemetry.addData("Target Position" , target);
            telemetry.addData("System VeloBelo" , actualVelo);
            telemetry.addData("Error" , controller.getError());
            telemetry.addData("System is Stuck" , controller.isStuck());
            telemetry.addData("System is busy" , controller.isBusy());
            telemetry.update();
        }
    }
    private double handleAngle(){
        double pos = encoder.getPosition();
        double raw =  pos%tpr;
        if(raw < 0)raw += tpr;
        return (raw/tpr)*360;
    }
    private void updateKernel(){
        kernel = new HighController.Kernel.Builder()
                .setPID(kP,kI,kD)
                .setFeedforward(staticGain,dynamicGain,inertialGain)
                .setBangBangRange(bangBangZone)
                .setTolerance(tolerance)
                .setPrecisionThreshold(precisionZone)
                .setSlewRate(slewRate)
                .setOscillationDampener(lookAhead,breakingGain)
                .setOpposingCorrection(useOpposingCorrection,spinOpposingGain,spinOpposingPrefix)
                .setBangBangGain(bangBangGain)
                .setPrecisionAddition(additionMin,additionMax)
                .setViscousGain(viscousGain)
                .build();
        controller.setKernel(kernel);
    }
}
