package org.firstinspires.ftc.teamcode.OpModes.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Core.Module.Outtake.Shooter;

@Config
@TeleOp(name = "Shooter Calibration", group = "Tests")
public class ShooterCalibration extends LinearOpMode {
    public static double targetVelocity = 0;
    public static double p = 0.002;
    public static double i = 0.002;
    public static double d = 0.00002;
    public static double f = 0.00010;
    public static double encoderResolution= 28;

    private Shooter shooter;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        shooter = new Shooter(hardwareMap);
        p = Shooter.kp;
        i = Shooter.ki;
        d = Shooter.kd;
        f = Shooter.kf;
        encoderResolution = Shooter.encoderResolution;
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        shooter.motorUp.setTolerance(0.15); //  m/s !!!!!!!! Cel putin , sper.
        waitForStart();
        while (opModeIsActive()) {
            Shooter.kp = p;
            Shooter.ki = i;
            Shooter.kd = d;
            Shooter.kf = f;
            Shooter.encoderResolution = encoderResolution;
            shooter.motorUp.setEncoderResolution(Shooter.encoderResolution);
            shooter.setTargetVelocity(targetVelocity);
            shooter.update();
            double currentVelo = shooter.motorUp.getCurrentVelocity();
            double currentPower = shooter.motorUp.getPower();
            double error = targetVelocity - currentVelo;
            telemetry.addData("Target Velocity", targetVelocity);
            telemetry.addData("Actual Velocity", currentVelo);
            telemetry.addData("Error", error);
            telemetry.addData("Velocity Error from motor", shooter.motorUp.pidfVelocity.getVelocityError());
            telemetry.addData("Possition Error from motor", shooter.motorUp.pidfVelocity.getPositionError());
            telemetry.addData("Motor Power", currentPower);
            telemetry.addData("PIDF Coeffs", String.format("P:%.5f I:%.5f D:%.5f F:%.5f", p, i, d, f));
            telemetry.addData("Is at target(Shooter Method)", shooter.atTarget());
            telemetry.addData("Is at target(Motor Optimised Method)", shooter.motorUp.atTarget());
            telemetry.update();
        }
    }
}