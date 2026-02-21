package org.firstinspires.ftc.teamcode.Core;

import static org.firstinspires.ftc.teamcode.Constants.Color.Blue;
import static org.firstinspires.ftc.teamcode.Constants.Color.Green;
import static org.firstinspires.ftc.teamcode.Constants.Color.Purple;
import static org.firstinspires.ftc.teamcode.Constants.Color.Red;
import static org.firstinspires.ftc.teamcode.Constants.Color.Yellow;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Core.Hardware.HighCamera;
import org.firstinspires.ftc.teamcode.Core.Hardware.HighModule;
import org.firstinspires.ftc.teamcode.Core.Module.Intake.IntakeMotor;
import org.firstinspires.ftc.teamcode.Core.Module.Intake.Intake;

import org.firstinspires.ftc.teamcode.Core.Module.Others.Led;
import org.firstinspires.ftc.teamcode.Core.Module.Outtake.Outtake;

import java.util.List;

public class Robot extends HighModule {

    protected HardwareMap hardwareMap;
    ElapsedTime timerShoot = new ElapsedTime(), voltageTimer = new ElapsedTime();
    Telemetry telemetry;

    public States state = States.Collect;
    public Actions lastAction = Actions.None;
    public Constants.Color allianceColor;

    public Follower drive;
    public Outtake outtake;
    public Intake intake;
    public HighCamera camera;
    public Led led;
    public List<LynxModule> allHubs;

    boolean isAuto;
    public boolean shootingSequence = false, holdingSequence = false, shouldAlignTurret = true;
    private Pose cameraPose = new Pose(0, 0, 0);
    int shootingState = 0;

    public int cycles = 0;

    public enum States {
        Collect,
        Shooting
    }

    public enum Actions {
        Shoot,
        StopShoot,
        None
    }

    public Robot(HardwareMap hardwareMap, Pose startPose, boolean isAuto, Constants.Color allianceColor, Telemetry telemetry, Gamepad gamepad) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        this.isAuto = isAuto;
        this.allianceColor = allianceColor;
        drive = Constants.createFollower(hardwareMap);
        camera = new HighCamera(hardwareMap, HighCamera.Pipelines.AprilTagId);
        drive.setStartingPose(startPose);
        drive.setPose(startPose);
        if (!isAuto) {
            if (allianceColor == Blue) {
                drive.startFieldCentricDrive(gamepad, true, Math.PI);
            } else {
                drive.startFieldCentricDrive(gamepad, true, 0);
            }
        }
        outtake = new Outtake(hardwareMap, allianceColor, telemetry);
        intake = new Intake(hardwareMap);
        led = new Led(hardwareMap);
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    public Robot(HardwareMap hardwareMap, Pose startPose, boolean isAuto, Constants.Color allianceColor, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        this.isAuto = isAuto;
        this.allianceColor = allianceColor;
        drive = Constants.createFollower(hardwareMap);
        camera = new HighCamera(hardwareMap, HighCamera.Pipelines.AprilTagId);
        drive.setStartingPose(startPose);
        outtake = new Outtake(hardwareMap, allianceColor, telemetry);
        intake = new Intake(hardwareMap);
        led = new Led(hardwareMap);
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    public void resetWithCamera() {
        drive.setStartingPose(new Pose(6, 6, 0));
        cameraPose = camera.getMegaTagFieldPose(outtake.turret.getCurrentAngleWrappedDegrees());
        if(cameraPose != null ){
            drive.setPose(cameraPose);
            outtake.turret.setOffset(0);
        }else {
            cameraPose = new Pose(-99,-99,-99);
        }

    }

    public void setAction(Actions action) {
        lastAction = action;
        switch (action) {
            case Shoot:
                shootingSequence = true;
                intake.canStop = false;
                shootingState = 0;
                outtake.shooter.disableCompensation();
                outtake.isShooting = true;
                outtake.shooter.wasAtTarget = false;
                timerShoot.reset();
                break;
            case StopShoot:
                shootingSequence = false;
                intake.canStop = true;
                shootingState = 0;
                break;
            case None:
                break;
        }
    }

    public boolean isDone() {
        return !drive.isBusy();
    }

    @Override
    public void update() {
        if (voltageTimer.milliseconds() >= 500) {
            Constants.Globals.voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
            voltageTimer.reset();
        }

        if (shootingSequence) {
            switch (shootingState) {
                case 0:
                    if (outtake.atTarget()) {
                        outtake.openBlocker();
                        intake.canStop = false;
                        shootingState++;
                        cycles = 1;
                        timerShoot.reset();
                    }
                    break;
                case 1:
                    if (cycles <= 3 || holdingSequence) {
                        if (outtake.atTargetCompensated() || (cycles <= 1 && outtake.atTarget())) {
                            intake.setPower(IntakeMotor.States.Collect);
                            intake.intakeMotor.update();
                            shootingState++;
                            timerShoot.reset();
                        }
                    } else {
                        intake.setPower(IntakeMotor.States.Wait);
                        outtake.shooter.setUpTargetVelocity(outtake.shooter.getTargetDown());
                        outtake.resetErrorTolerance();
                        outtake.shooter.disableCompensation();
                        outtake.isShooting = false;
                        outtake.closeBlocker();
                        outtake.shooter.wasAtTarget = false;
                        intake.canStop = true;
                        shootingSequence = false;
                        shootingState = -1;
                        cycles = -1;
                    }
                    break;
                case 2:
                    boolean checkShoot = outtake.shooter.detectShoot();
                    boolean ballFired = (outtake.hasShot || checkShoot); //|| timerShoot.milliseconds() >= 450
                    if (ballFired) {
                        if (cycles <= 3) {
                            outtake.addErrorToleranceScaled();
                        }
                        if (!outtake.atTargetCompensated()) {
                            intake.setPower(IntakeMotor.States.Wait);
                            intake.intakeMotor.update();
                        }
                        outtake.shooter.enableCompensation();
                        shootingState = 1;
                        cycles++;
                    } else if (timerShoot.milliseconds() > 1000) {
                        if (!outtake.atTargetCompensated()) {
                            intake.setPower(IntakeMotor.States.Wait);
                            intake.intakeMotor.update();
                        }
                        outtake.shooter.enableCompensation();
                        shootingState = 1;
                        cycles++;
                    }
                    break;
            }
        }
        intake.update();
        drive.update();
        outtake.update(drive.getPose());
        if (shouldAlignTurret) {
            outtake.alignTurret();
        }
        led.update();
//        telemetry.addData("Freq pinpoint", drive.poseTracker.localizer);
        telemetry.addData("States", shootingState);
        telemetry.addData("Jerk ", outtake.shooter.jerk);
        telemetry.addData("Ball fired status", outtake.hasShot);
        telemetry.addData("atTargetCompensated", outtake.shooter.atTargetCompensated());
        telemetry.addData("Camera pose ", cameraPose.toString());
        telemetry.addData("atTarget", outtake.shooter.atTarget());
        if (state == States.Collect) {
            if (intake.isFull) {
                led.setColor(Green);
            } else if (intake.isPartial) {
                led.setColor(Yellow);
            } else {
                led.setColor(Red);
            }
        } else {
            if (outtake.atTarget()) {
                led.setColor(Green);
            } else {
                led.setColor(Purple);
            }
        }

        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
    }

}
