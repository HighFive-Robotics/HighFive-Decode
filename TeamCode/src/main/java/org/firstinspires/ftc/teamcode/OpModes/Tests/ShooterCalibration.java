package org.firstinspires.ftc.teamcode.OpModes.Tests;

import static org.firstinspires.ftc.teamcode.Constants.DeviceNames.intakeMotorName;
import static org.firstinspires.ftc.teamcode.Constants.Globals.autoColor;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Core.Hardware.HighMotor;
import org.firstinspires.ftc.teamcode.Core.Module.Outtake.Blocker;
import org.firstinspires.ftc.teamcode.Core.Module.Outtake.Shooter;
import org.firstinspires.ftc.teamcode.Core.Module.Outtake.Turret;

@Config
@TeleOp(name = "Shooter Calibration", group = "Tests")
public class ShooterCalibration extends LinearOpMode {

    Pose BlueGoal = new Pose(16,132);
    Pose RedGoal = new Pose(128,132);

    enum Mode{
        Up,

        Down,
        Test,
    }
    public static Mode mode = Mode.Test;
    DcMotorEx motor;
    public static double targetVelocity = 0, compensation = 1;
    public Shooter shooter;
    public Turret turret;
    public boolean shoot;
    public int k;
    public int cycles;
    Follower drive;
    boolean shootingSeq=false;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        shooter = new Shooter(hardwareMap);
        drive = Constants.createFollower(hardwareMap);
        turret = new Turret(hardwareMap, Constants.Color.Red);
        drive.setStartingPose(new Pose(6,6,0));
        drive.startFieldCentricDrive(gamepad1, true, 0);
        motor = hardwareMap.get(DcMotorEx.class, intakeMotorName);
        double tolerance;
        telemetry.addLine("Init");
        waitForStart();
        turret.setTarget(0);
        while (opModeIsActive()){
            if(targetVelocity >= 3.2)tolerance = 0.25;
            switch (mode) {
                case Down:
                    shooter.setDownTargetVelocity(targetVelocity);
                    shooter.nanUp();
                    shooter.updateCoefDown();

                    break;
                case Up:
                    shooter.setUpTargetVelocity(targetVelocity);
                    shooter.nanDown();
                    shooter.updateCoefUp();
                    break;
                case Test:
                    shooter.setVelocity(targetVelocity,compensation);
                    break;
            }
            if(gamepad1.xWasPressed()){
                shootingSeq = true;
                k=0;
            }
            if(gamepad1.leftBumperWasPressed()) shooter.blocker.setState(Blocker.States.Open);
            if(gamepad1.rightBumperWasPressed()) shooter.blocker.setState(Blocker.States.Close);
//            if(gamepad1.dpadRightWasPressed())turret.setTarget(turret.getTarget()+25);
//            if(gamepad1.dpadLeftWasPressed())turret.setTarget(turret.getTarget()-25);
            if(gamepad1.dpadLeftWasPressed()) {
                turret.addOffsetDegrees(-2);
            }
            if(gamepad1.dpadRightWasPressed()) {
                turret.addOffsetDegrees(+2);
            }
            if(gamepad1.psWasPressed()) drive.resetTeleOpHeading();
            if(!shootingSeq){
                if(gamepad1.right_trigger >= 0.4){
                    motor.setPower(1);
                }else if(gamepad1.left_trigger >= 0.4){
                    motor.setPower(-1);
                }else{
                    motor.setPower(0);
                }
            }
            turret.setTarget(turret.getTargetAngleFromDistance(drive.getPose()));
            telemetry.addData("Target Velocity",targetVelocity);
            telemetry.addData("Distance From Goal", getDistance());
            telemetry.addData("Power", shooter.motorDown.getPower());
            telemetry.addData("Pose " , drive.getPose());
            if(shootingSeq){
                switch (k){
                    case 0:
                        shooter.blocker.setState(Blocker.States.Open);
                        k++;
                    case 1:
                        if(shooter.atTarget()){
                            motor.setPower(1);
                            k++;
                        }
                    case 2:
                        if(shooter.getVelocityErrorDown() >= 0.25){
                            shooter.blocker.setState(Blocker.States.Close);
                            k = 0;
                            shootingSeq = false;
                        }
                }
            }
            turret.update();
            shooter.update();
            drive.update();
            turret.update();
            telemetry.update();
        }
    }

    private double getDistance(){
        switch (autoColor){
            case Blue:
                return Math.hypot(BlueGoal.getX() - drive.getPose().getX(),BlueGoal.getY() - drive.getPose().getY());
            case Red:
                return Math.hypot(RedGoal.getX() - drive.getPose().getX(),RedGoal.getY()-drive.getPose().getY());
        }
        return -1;
    }
}