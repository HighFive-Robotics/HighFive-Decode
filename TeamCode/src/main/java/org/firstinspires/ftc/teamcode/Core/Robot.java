package org.firstinspires.ftc.teamcode.Core;

import static org.firstinspires.ftc.teamcode.Constants.Globals.autoColor;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Core.Hardware.HighModule;
import org.firstinspires.ftc.teamcode.Core.Module.Others.Drive;
import org.firstinspires.ftc.teamcode.Core.Module.Intake.Intake;
import org.firstinspires.ftc.teamcode.Core.Module.Others.Lift;
import org.firstinspires.ftc.teamcode.Core.Module.Outtake.Shooter;

import java.util.List;

public class Robot extends HighModule {

    ElapsedTime timerShoot = new ElapsedTime(), timerIntake = new ElapsedTime();

    Telemetry telemetry;
    public Actions lastAction = Actions.None;
    public Follower drive;
    public List<LynxModule> allHubs;
    protected HardwareMap hardwareMap;

    public IMU imu;

    //public Camera camera;
    public Shooter shooter;
    public Intake intake;
    public Lift lift;

    boolean isAuto;
    boolean stopShoot = false, stopIntake = false;

    public enum Actions {
        Shoot,
        PrepareForShooting,

        WaitToBeFedUp,
        ColorGreen,//Doesn't work
        ColorPurple,//Doesn't work
        None
    }

    public Robot(HardwareMap hardwareMap, Pose startPose, boolean isAuto, Constants.Color allianceColor, Telemetry telemetry , Gamepad gamepad) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        this.isAuto = isAuto;
        drive = Constants.createFollower(hardwareMap);
        if (isAuto) {
            drive.setStartingPose(startPose);
            //camera = new Camera(hardwareMap);
        } else {
            if(autoColor == Constants.Color.Blue) {
                drive.startFieldCentricDrive(gamepad, true, Math.PI + startPose.getHeading());
            } else {
                drive.startFieldCentricDrive(gamepad, true, startPose.getHeading());
            }
        }
        lift = new Lift(hardwareMap);
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

    }

    public Robot(HardwareMap hardwareMap, Pose startPose, boolean isAuto, Constants.Color allianceColor, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        this.isAuto = isAuto;
        drive = Constants.createFollower(hardwareMap);
        if (isAuto) {
            drive.setStartingPose(startPose);
           // camera = new Camera(hardwareMap);
        }
        lift = new Lift(hardwareMap);
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

    }

    public void setAction(Actions action){
        lastAction = action;
        switch (action){
            case Shoot:
                intake.setAction(Intake.IntakeActions.Collect);
                stopShoot = true;
                timerShoot.reset();
                break;
            case PrepareForShooting:
                intake.intakeMotor.setPower(-0.7);
                stopIntake = true;
                timerIntake.reset();
                break;
        }
    }

    public boolean isDone() {
        return !drive.isBusy();
    }

    @Override
    public void update() {
        Constants.Globals.voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();

        if(stopShoot && (timerShoot.milliseconds() > 1200 || shooter.getVelocityError() >= 0.7)){
            stopShoot = false;
            intake.setAction(Intake.IntakeActions.Wait);
        }

        if(stopIntake && timerIntake.milliseconds() >= 450){
            stopIntake = false;
            intake.setAction(Intake.IntakeActions.Wait);
        }

        shooter.update();
        intake.update();
        lift.update();
        drive.update();
        if (isAuto) {
            //camera.update();
        }
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
    }
}
