package org.firstinspires.ftc.teamcode.Core;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Core.Hardware.HighModule;
import org.firstinspires.ftc.teamcode.Core.Module.Camera.Camera;
import org.firstinspires.ftc.teamcode.Core.Module.Drive;
import org.firstinspires.ftc.teamcode.Core.Module.Intake.Intake;
import org.firstinspires.ftc.teamcode.Core.Module.Outtake.Shooter;

import java.util.List;

public class Robot extends HighModule {

    Telemetry telemetry;
    public Actions lastAction = Actions.None;
    public Follower drive;
    public Drive teleOpDrive;
    public List<LynxModule> allHubs;
    protected HardwareMap hardwareMap;

    public IMU imu;

    public Camera camera;
    public Shooter shooter;
    public Intake intake;

    boolean isAuto;

    public enum Actions {
        WaitToBeFedUp,
        ColorGreen,//Doesn't work
        ColorPurple,//Doesn't work
        None
    }

    public Robot(HardwareMap hardwareMap, Pose startPose, boolean isAuto, Constants.Color allianceColor, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        this.isAuto = isAuto;
        if (isAuto) {
            drive = Constants.createFollower(hardwareMap);
            drive.setStartingPose(startPose);
            camera = new Camera(hardwareMap);
        } else {
            teleOpDrive = new Drive(hardwareMap);
        }
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

    }

    ElapsedTime failSafeTimer = new ElapsedTime();

    public boolean isDone() {
        return !drive.isBusy();
    }

    @Override
    public void update() {

        if (isAuto) {
            drive.update();
            camera.update();
        } else {
            teleOpDrive.update();
        }

        shooter.update();
        intake.update();

        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
    }
}
