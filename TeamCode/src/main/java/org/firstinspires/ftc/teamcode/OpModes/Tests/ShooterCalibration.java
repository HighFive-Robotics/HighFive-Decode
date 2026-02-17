package org.firstinspires.ftc.teamcode.OpModes.Tests;

import static org.firstinspires.ftc.teamcode.Constants.DeviceNames.intakeMotorName;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Core.Hardware.HighCamera;
import org.firstinspires.ftc.teamcode.Core.Module.Outtake.Outtake;

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
    public int shootingState;
    public int cycles;
    Follower drive;
    boolean shootingSequence=false , holdingSequence = false;
    public static boolean dacia = true;
    ElapsedTime timerShoot;
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
        timerShoot = new ElapsedTime();
        double tolerance;
        telemetry.addLine("Init");
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.yWasPressed()) {
                shootingSequence = true;
                holdingSequence = true;
                shootingState = 0;
            }
            if(gamepad1.yWasReleased()){
                holdingSequence = false;
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
            if (!shootingSequence) {
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

            if (shootingSequence) {
                switch (shootingState) {
                    case 0:
                        outtake.openBlocker();
                        outtake.shooter.enableCompensation();
                        shootingState++;
                        cycles = 1;
                        timerShoot.reset();
                        break;
                    case 1:
                        if (cycles <= 3 || holdingSequence) {
                            if (outtake.atTarget()) {
                                motor.setPower(1);
                                if(cycles <= 3){
                                    outtake.shooter.addToleranceCompensationOffset(0.25);
                                }
                                shootingState++;
                                timerShoot.reset();
                            }
                        } else {
                            motor.setPower(0);
                            outtake.shooter.setUpTargetVelocity(outtake.shooter.getTargetDown());
                            outtake.resetErrorTolerance();
                            outtake.shooter.disableCompensation();
                            outtake.closeBlocker();
                            shootingSequence = false;
                            shootingState = -1;
                            cycles = -1;
                        }
                        break;
                    case 2:
                        boolean ballFired = outtake.hasShot || timerShoot.milliseconds() >= 275 || outtake.checkErrorToleranceDown(0.25);
                        if(ballFired) {
                            if(!outtake.atTarget()){
                                motor.setPower(0);
                            }
                            cycles++;
                            shootingState = 1;
                        } else if (timerShoot.milliseconds() > 1000) {
                            if(!outtake.atTarget()){
                                motor.setPower(0);
                            }
                            cycles++;
                            shootingState = 1;
                        }
                        break;
                }
            }
            if(dacia && mode == Mode.None){
                outtake.shooter.setManualVelocity(velocityDown);
            }
            if(mode == Mode.Up){
                outtake.shooter.setUpTargetVelocity(velocityUp);
                outtake.shooter.setDownTargetVelocity(velocityDown);
                if(!shootingSequence){
                    outtake.shooter.disableCompensation();
                }
            }
            outtake.update(drive.getPose());
            outtake.alignTurret();
            outtake.debug();
            telemetry.addData("holding" , holdingSequence);
            telemetry.addData("Robot Pose" , drive.getPose());
            telemetry.addData("shouldCompensate " , outtake.shooter.shouldCompensate);
            drive.update();
            telemetry.update();
        }
    }
}