package org.firstinspires.ftc.teamcode.Core.Module.Outtake;

import static org.firstinspires.ftc.teamcode.Constants.DeviceNames.hoodServoName;
import static org.firstinspires.ftc.teamcode.Constants.OuttakeConstants.HoodConstants.initAngle;
import static org.firstinspires.ftc.teamcode.Constants.OuttakeConstants.HoodConstants.maxAngle;
import static org.firstinspires.ftc.teamcode.Constants.OuttakeConstants.HoodConstants.maxServoPos;
import static org.firstinspires.ftc.teamcode.Constants.OuttakeConstants.HoodConstants.minAngle;
import static org.firstinspires.ftc.teamcode.Constants.OuttakeConstants.HoodConstants.minServoPos;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Core.Hardware.HighModule;
import org.firstinspires.ftc.teamcode.Core.Hardware.HighServo;

@Config
public class Hood extends HighModule {

    public HighServo hoodServo;
    private double targetAngle;
    private double offsetDeg = 0;
    public Hood(HardwareMap hwMap, boolean isAuto) {
        this.targetAngle = initAngle;
        hoodServo = HighServo.Builder.startBuilding()
                .setServo(hwMap.get(Servo.class, hoodServoName))
                .setStandardRunMode()
                .setInitPosition(angleToPosition(initAngle), isAuto)
                .build();
    }
    public Hood(HardwareMap hwMap, boolean isAuto , double initAngle) {
        this.targetAngle = initAngle;
        hoodServo = HighServo.Builder.startBuilding()
                .setServo(hwMap.get(Servo.class, hoodServoName))
                .setStandardRunMode()
                .setInitPosition(angleToPosition(initAngle), isAuto)
                .build();
    }
    public void setAngle(double angle) {
        this.targetAngle = Math.max(minAngle, Math.min(maxAngle, angle));
        hoodServo.setPosition(angleToPosition(this.targetAngle));
    }
    public double getAngle() {
        return this.targetAngle;
    }
    public double getAngleWithOffset() {
        return this.targetAngle + offsetDeg;
    }
    private double angleToPosition(double angle) {
        return minServoPos + ((angle - minAngle) * (maxServoPos - minServoPos) / (maxAngle - minAngle));
    }
    public void addOffsetDegrees(double angle){
        offsetDeg += angle;
        setAngle(getAngle() + offsetDeg);
    }
    public void subtractOffsetDegrees(double angle){
        offsetDeg -= angle;
        setAngle(getAngle() + offsetDeg);
    }
    public void setOffsetDegrees(double angle){
        offsetDeg = angle;
        setAngle(getAngle() + offsetDeg);
    }
    public void resetOffsetDegrees(double angle){
        offsetDeg = 0;
        setAngle(getAngle());
    }
    @Override
    public void update() {
        hoodServo.update();
    }
}