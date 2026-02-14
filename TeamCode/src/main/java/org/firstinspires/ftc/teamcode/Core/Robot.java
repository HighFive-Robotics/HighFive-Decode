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
    public boolean shootingSequence = false , holdingSequence = false, shouldAlignTurret = true;
    int shootingState = 0;

    public int cycles = 0;

    public enum States {
        Collect,
        Shooting
    }

    public enum Actions {
        Shoot,
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
                if(allianceColor == Blue){
                    drive.startFieldCentricDrive(gamepad, true, Math.PI);
                }else {
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

    public void setAction(Actions action) {
        lastAction = action;
        switch (action) {
            case Shoot:
                shootingSequence = true;
                intake.canStop = false;
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
                    outtake.openBlocker();
                    cycles = 1;
                    timerShoot.reset();
                    shootingState++;
                    break;
                case 1:
                    if (cycles <= 3 || holdingSequence) {
                        if (outtake.atTarget() || (cycles == 1 && timerShoot.milliseconds()>=400)) {
                            intake.setPower(IntakeMotor.States.Collect);
                            timerShoot.reset();                            shootingState++;
                            if(cycles <= 3){
                                outtake.increaseToleranceOffset(0.07,0.03);
                            }
                        }
                    } else {
                        cycles = -1;
                        state = States.Collect;
                        intake.canStop = true;
                        shootingState = -1;
                        outtake.closeBlocker();
                        outtake.setToleranceOffset(0,0);
                        shootingSequence = false;
                        intake.setPower(IntakeMotor.States.Wait);
                    }
                    break;
                case 2:
                    boolean ballFired = outtake.hasShot ||  timerShoot.milliseconds() >= 275 || outtake.checkErrorTolerance(0.7,0.2);
                    boolean minPulseCheck = timerShoot.milliseconds() > 25;
                    if(ballFired && minPulseCheck) {
                        intake.setPower(IntakeMotor.States.Wait);
                        cycles++;
                        shootingState = 1;
                    } else if (timerShoot.milliseconds() > 1000) {
                        intake.setPower(IntakeMotor.States.Wait);
                        cycles++;
                        shootingState = 1;
                    }
                    break;
            }
        }

        intake.update();
        drive.update();
        outtake.update(drive.getPose());
        if(shouldAlignTurret){
            outtake.alignTurret();
        }
        led.update();

        telemetry.addData("isPartial", intake.isPartial);
        telemetry.addData("isFull", intake.isFull);
        telemetry.addData("BB1", intake.breakBeamUp.getState());
        telemetry.addData("BB2", intake.breakBeamDown.getState());

        if(state == States.Collect){
            if(intake.isFull){
                led.setColor(Green);
            } else if(intake.isPartial){
                led.setColor(Yellow);
            } else {
                led.setColor(Red);
            }
        } else {
            if(outtake.atTarget()){
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
