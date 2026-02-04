package org.firstinspires.ftc.teamcode.OpModes.Tests;

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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Core.Module.Outtake.Blocker;
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
    public static Mode mode = Mode.None;
    DcMotorEx motor;
    public static double velocityUp = 0, velocityDown = 0;
    public Shooter shooter;
    public Turret turret;
    public boolean shoot;
    public int k;
    public int cycles;
    Follower drive;
    boolean shootingSeq=false;
    ElapsedTime timer;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        shooter = new Shooter(hardwareMap);
        drive = Constants.createFollower(hardwareMap);
        turret = new Turret(hardwareMap, Constants.Color.Red);
        drive.setStartingPose(new Pose(6,6,0));
        drive.startFieldCentricDrive(gamepad1, true, 0);
        motor = hardwareMap.get(DcMotorEx.class, intakeMotorName);
        timer = new ElapsedTime();
        double tolerance;
        telemetry.addLine("Init");
        waitForStart();
        turret.setTarget(0);
        while (opModeIsActive()){

            switch (mode) {
                case Down:
                    shooter.setDownTargetVelocity(velocityDown);
                    shooter.nanUp();
                    shooter.updateCoefficientsDown();

                    break;
                case Up:
                    shooter.setUpTargetVelocity(velocityUp);
                    shooter.nanDown();
                    shooter.updateCoefficientsUp();
                    break;
                case Test:
                    shooter.setDownTargetVelocity(velocityDown);
                    shooter.setUpTargetVelocity(velocityUp);
                    break;
            }
            if(gamepad1.yWasPressed()){
                shootingSeq = true;
                k=0;
            }
            if(gamepad1.leftBumperWasPressed()) shooter.blocker.setState(Blocker.States.Open);
            if(gamepad1.rightBumperWasPressed()) shooter.blocker.setState(Blocker.States.Close);
            if(gamepad1.dpadLeftWasPressed()) {
                turret.addOffsetDegrees(-5);
            }
            if(gamepad1.dpadRightWasPressed()) {
                turret.addOffsetDegrees(+5);
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
            if (gamepad1.square) {
                shooter.setTargetVelocity(getDistance());
            }

            if(shootingSeq){
                switch (k){
                    case 0:
                        shooter.blocker.setState(Blocker.States.Open);
                        cycles = 1;
                        k++;
                        break;
                    case 1:
                        if(cycles <= 3){
                            if(shooter.atTarget()){
                                motor.setPower(1);
                                timer.reset();
                                k++;
                                shooter.addToUpToleranceOffset(0.025);
                                shooter.addToDownToleranceOffset(0.06);
                            }
                        } else {
                            cycles = -1;
                            k = -1;
                            shooter.setDownToleranceOffset(0);
                            shooter.setUpToleranceOffset(0);
                            shootingSeq = false;
                            shooter.blocker.setState(Blocker.States.Close);
                            motor.setPower(0);
                        }
                        break;
                    case 2:
                        boolean ballFired = (shooter.getVelocityErrorDown() >= 0.35 || shooter.getVelocityErrorUp() >= 0.1) || timer.milliseconds() >= 450;
                        boolean minPulseCheck = timer.milliseconds() > 25;
                        if(ballFired && minPulseCheck){
                            motor.setPower(0);
                            cycles++;
                            k = 1;
                        }
                        else if (timer.milliseconds() > 1000) {
                            motor.setPower(0);
                            cycles++;
                            k = 1;
                        }
                        break;
                }
            }
            if(gamepad1.optionsWasPressed()){
                shooter.motorDown.motor.setPower(0.5);
            }
            telemetry.addData("Down Velocity",shooter.motorDown.getCurrentVelocity());
            telemetry.addData("Up Velocity",shooter.motorUp.getCurrentVelocity());
            telemetry.addData("State shoot",k);
            telemetry.addData("Bool shoot",shootingSeq);
            telemetry.addData("error velo" , shooter.getVelocityErrorDown());
            telemetry.addData("Distance From Goal", getDistance());
            telemetry.addData("Power", shooter.motorDown.getPower());
            telemetry.addData("Pose " , drive.getPose());
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
                return 2.54 * Math.hypot(BlueGoalDistance.getX() - drive.getPose().getX(),BlueGoalDistance.getY() - drive.getPose().getY());
            case Red:
                return 2.54 * Math.hypot(RedGoalDistance.getX() - drive.getPose().getX(),RedGoalDistance.getY() - drive.getPose().getY());
        }
        return -1;
    }
}