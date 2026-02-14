package org.firstinspires.ftc.teamcode.OpModes.Tests;

import static org.firstinspires.ftc.teamcode.Constants.DeviceNames.breakBeamOuttakeName;
import static org.firstinspires.ftc.teamcode.Constants.DeviceNames.intakeMotorName;
import static org.firstinspires.ftc.teamcode.Constants.Globals.BlueGoalDistance;
import static org.firstinspires.ftc.teamcode.Constants.Globals.RedGoalDistance;
import static org.firstinspires.ftc.teamcode.Constants.Globals.autoColor;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Core.Hardware.HighCamera;
import org.firstinspires.ftc.teamcode.Core.Module.Outtake.Blocker;
import org.firstinspires.ftc.teamcode.Core.Module.Outtake.Outtake;
import org.firstinspires.ftc.teamcode.Core.Module.Outtake.Shooter;
import org.firstinspires.ftc.teamcode.Core.Module.Outtake.Turret;

@Config
@TeleOp(name = "Shooter Calibration", group = "Tests")
public class ShooterCalibration extends LinearOpMode {

    enum Mode{
        Up,
        Down,
        Test,
        None
    }

    HighCamera camera;
    public Outtake outtake;
    public static Mode mode = Mode.None;
    DcMotorEx motor;
    public static double velocityUp = 0, velocityDown = 0;
    public boolean shoot;
    public int k;
    public int cycles;
    Follower drive;
    boolean shootingSeq=false , holdingSeq = false;
    public static boolean dacia = true;
    ElapsedTime timer;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        camera = new HighCamera(hardwareMap, HighCamera.Pipelines.AprilTagLocation);
        camera.startCapture();
        drive = Constants.createFollower(hardwareMap);
        drive.setStartingPose(new Pose(6, 6, 0));
        drive.startFieldCentricDrive(gamepad1, true, 0);
        motor = hardwareMap.get(DcMotorEx.class, intakeMotorName);
        outtake = new Outtake(hardwareMap , Constants.Color.Red , telemetry);
        outtake.turret.reset();
        timer = new ElapsedTime();
        double tolerance;
        telemetry.addLine("Init");
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.yWasPressed()) {
                shootingSeq = true;
                holdingSeq = true;
                k = 0;
            }
            if(gamepad1.yWasReleased()){
                holdingSeq = false;
            }
            if (gamepad1.leftBumperWasPressed()) outtake.openBlocker();
            if (gamepad1.rightBumperWasPressed()) outtake.closeBlocker();
            if (gamepad1.dpadLeftWasPressed()) {
                outtake.offsetTurretToLeft(2.5);
            }
            if (gamepad1.dpadRightWasPressed()) {
                outtake.offsetTurretToRight(2.5);
            }
            if (gamepad1.psWasPressed()) drive.resetTeleOpHeading();
            if (gamepad1.psWasPressed()) drive.resetTeleOpHeading();
            if (!shootingSeq) {
                if (gamepad1.right_trigger >= 0.4) {
                    motor.setPower(1);
                } else if (gamepad1.left_trigger >= 0.4) {
                    motor.setPower(-1);
                } else {
                    motor.setPower(0);
                }
            }
            if (gamepad1.square) {
                outtake.setShootingVelocity();
            }
            if(gamepad1.dpad_up){
                outtake.setShootingVelocityOffset(-2);
            }
            if (gamepad1.circle) {
                outtake.shooter.updateAllCoefficients();
            }
            if (gamepad1.optionsWasPressed()) {
                dacia = !dacia;
            }

            if (shootingSeq) {
                switch (k) {
                    case 0:
                        outtake.openBlocker();
                        cycles = 1;
                        k++;
                        break;
                    case 1:
                        if (cycles <= 3 || holdingSeq) {
                            if (outtake.atTarget()) {
                                motor.setPower(1);
                                timer.reset();
                                k++;
                                if(cycles <= 3){
                                    outtake.increaseToleranceOffset(0.035,0.025);
                                }
                            }
                        } else {
                            cycles = -1;
                            k = -1;
                            outtake.closeBlocker();
                            outtake.setToleranceOffset(0,0);
                            shootingSeq = false;
                            motor.setPower(0);
                        }
                        break;
                    case 2:
                        boolean ballFired = (outtake.checkErrorTolerance(0.35,1)) || timer.milliseconds() >= 275;
                        boolean minPulseCheck = timer.milliseconds() > 25;
                        if (ballFired && minPulseCheck) {
                            motor.setPower(0);
                            cycles++;
                            k = 1;
                        } else if (timer.milliseconds() > 1000) {
                            motor.setPower(0);
                            cycles++;
                            k = 1;
                        }
                        break;
                }
            }
            if(dacia){
                outtake.shooter.setManualVelocity(velocityDown);
            }
            outtake.update(drive.getPose());
            outtake.alignTurret();
            outtake.debug();
            telemetry.addData("holding" , holdingSeq);
            telemetry.addData("Robot Pose" , drive.getPose());
            drive.update();
            telemetry.update();
        }
    }
}