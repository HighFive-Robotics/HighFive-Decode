package org.firstinspires.ftc.teamcode.OpModes.Tests;

import static org.firstinspires.ftc.teamcode.Constants.DeviceNames.shooterMotorUpName;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Core.Hardware.HighMotor;

@TeleOp(name = "Testing Velocity HIGHMOTOR")
@Config
public class HighMotorVelo extends LinearOpMode {
    HighMotor motor;
    public static double target=5;
    public static double kp=0.001,ki=0.002,kd=0,kf=0.00016;
    FtcDashboard dashboard;
    Telemetry graph;
    @Override
    public void runOpMode() throws InterruptedException {
        motor = new HighMotor(hardwareMap.get(DcMotorEx.class,shooterMotorUpName), HighMotor.RunMode.Velocity,false,true  ,false);
        motor.setVelocityPIDCoefficients(kp,ki,kd,kf);
        motor.setWheelDiameter(0.096);
        motor.setEncoderResolution(28);
        motor.setTolerance(20);
        dashboard = FtcDashboard.getInstance();
        graph = dashboard.getTelemetry();
        waitForStart();
        while (opModeIsActive()){
            motor.setTarget(target);
            motor.setVelocityPIDCoefficients(kp,ki,kd,kf);
            graph.addData("Target", motor.getTarget());
            graph.addData("Velo" , (motor.getVelocity()/motor.getEncoderResolution())*motor.getWheelDiameter());
            motor.update();
            graph.update();
        }
    }
}
