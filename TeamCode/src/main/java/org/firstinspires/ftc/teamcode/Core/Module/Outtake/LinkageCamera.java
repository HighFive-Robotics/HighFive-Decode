package org.firstinspires.ftc.teamcode.Core.Module.Outtake;

import static org.firstinspires.ftc.teamcode.Constants.DeviceNames.blockerServoName;
import static org.firstinspires.ftc.teamcode.Constants.DeviceNames.cameraServoName;
import static org.firstinspires.ftc.teamcode.Constants.OuttakeConstants.LinkageCamera.ArtifactPose;
import static org.firstinspires.ftc.teamcode.Constants.OuttakeConstants.LinkageCamera.GoalPose;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Core.Hardware.HighModule;
import org.firstinspires.ftc.teamcode.Core.Hardware.HighServo;

@Config
public class LinkageCamera extends HighModule {
    public HighServo servo;

    public enum States {
        Artifact,
        Goal
    }

    public States state;

    public LinkageCamera(HardwareMap hwMap, double initPosition, boolean isAuto) {
        servo = HighServo.Builder.startBuilding()
                .setServo(hwMap.get(Servo.class , cameraServoName))
                .setMotionProfilerRunMode()
                .setMotionProfilerCoefficients(3,4,2)
                .setInitPosition(initPosition, isAuto)
                .build();
    }

    public void setState(States state) {
        this.state = state;
        switch (state) {
            case Artifact:
                setTarget(ArtifactPose);
                break;
            case Goal:
                setTarget(GoalPose);
                break;
        }
    }

    public void setState(States state, double time) {
        this.state = state;
        switch (state) {
            case Artifact:
                setTarget(ArtifactPose, time);
                break;
            case Goal:
                setTarget(GoalPose, time);
                break;
        }
    }

    public void setTarget(double target) {
        this.target = target;
        servo.setPosition(target);
    }

    public void setTarget(double target, double time) {
        this.target = target;
        servo.setPosition(target, time);
    }

    @Override
    public boolean atTarget() {
        return servo.atTarget();
    }

    public double getTarget() {
        return target;
    }

    @Override
    public void update() {
        servo.update();
    }
}