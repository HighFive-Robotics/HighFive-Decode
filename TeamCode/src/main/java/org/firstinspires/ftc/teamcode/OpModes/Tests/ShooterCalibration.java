package org.firstinspires.ftc.teamcode.OpModes.Tests;

import static org.firstinspires.ftc.teamcode.Constants.DeviceNames.intakeMotorName;
import static org.firstinspires.ftc.teamcode.Constants.DeviceNames.turretMotorName;
import static org.firstinspires.ftc.teamcode.Constants.OuttakeConstants.TurretParams.kdTurret;
import static org.firstinspires.ftc.teamcode.Constants.OuttakeConstants.TurretParams.kfTurret;
import static org.firstinspires.ftc.teamcode.Constants.OuttakeConstants.TurretParams.kiTurret;
import static org.firstinspires.ftc.teamcode.Constants.OuttakeConstants.TurretParams.kpTurret;

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

@Config
@TeleOp(name = "Shooter Calibration", group = "Tests")
public class ShooterCalibration extends LinearOpMode {

    HighMotor turret;

    enum Mode{
        Up,
        Down,
        Both,
        Angle,
        Test,
        None
    }
    public static Mode mode = Mode.None;
    DcMotorEx motor;
    public static double targetVelocity = 0, compensation = 0.6 ,sV=0;
    public Shooter shooter;
    public boolean shoot;
    public int k;
    public int cycles;
    Follower drive;
    boolean noSeq=true;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        shooter = new Shooter(hardwareMap);
        drive = Constants.createFollower(hardwareMap);
        drive.setStartingPose(new Pose(72,72,Math.PI));
        drive.startFieldCentricDrive(gamepad1, true, 0);
        motor = hardwareMap.get(DcMotorEx.class, intakeMotorName);
        double tolerance;
        turret = HighMotor.Builder.startBuilding()
                .setMotor(hardwareMap.get(DcMotorEx.class, turretMotorName))
                .setRunMode(HighMotor.RunMode.PID)
                .setReverseMotor(true)
                .setUseZeroPowerBehaviour(false)
                .setPIDCoefficients(kpTurret,kiTurret,kdTurret,kfTurret, HighMotor.FeedForwardType.Lift,1)
                .setEncoder(true,false)
                .build();
        telemetry.addLine("Init");
        waitForStart();
        turret.setTarget(0);
        if(gamepad2.dpadDownWasPressed()){
            turret.resetMotor();
        }
        while (opModeIsActive()){
            if(targetVelocity >= 3.2)tolerance = 0.4;
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
                case Both:
                    shooter.setTargetVelocity(targetVelocity, sV);
                    shooter.updateAllCoef();

                    break;
                case Angle:
                    shooter.setAntiBackSpinVelocity(targetVelocity);
                    shooter.updateAllCoef();
                    break;
                case Test:
                    shooter.setVelocity(targetVelocity,compensation);
                    break;
                case None: ;

                    if(gamepad1.aWasPressed())shooter.setAntiBackSpinVelocity(2.5);
                    if(gamepad1.bWasPressed())shooter.setAntiBackSpinVelocity(2.8);
                    if(gamepad1.xWasPressed())shooter.setAntiBackSpinVelocity(3.9);
                    if(gamepad1.yWasPressed())shooter.setAntiBackSpinVelocity(0);
                    if(gamepad1.dpadDownWasPressed())shooter.setAntiBackSpinVelocity(4);
                    break;
            }
            if(gamepad1.leftBumperWasPressed()) shooter.blocker.setState(Blocker.States.Open);
            if(gamepad1.rightBumperWasPressed()) shooter.blocker.setState(Blocker.States.Close);
            if(gamepad1.psWasPressed()) drive.resetTeleOpHeading();

            if(noSeq){
                if(gamepad1.right_trigger >= 0.4){
                    motor.setPower(1);
                }else if(gamepad1.left_trigger >= 0.4){
                    motor.setPower(-1);
                }else motor.setPower(0);
            }
            if(gamepad1.optionsWasPressed()){
                shoot = true;
                noSeq = false;
                cycles = 0;
                k=0;
            }
            if(gamepad1.dpadRightWasPressed())turret.setTarget(turret.getTarget()+25);
            if(gamepad1.dpadLeftWasPressed())turret.setTarget(turret.getTarget()-25);
            if(gamepad1.xWasPressed())shooter.setAntiBackSpinVelocity(3.9);
            if(shoot){
                switch (k){
                    case 0:
                        shooter.blocker.setState(Blocker.States.Open);
                        k++;
                        break;
                    case 1:

                        if(shooter.atTarget() && cycles < 3){
                            motor.setPower(1);
                            k++;
                        }else if(cycles >= 3){
                            shooter.blocker.setState(Blocker.States.Close);
                            shoot=false;
                            noSeq=true;
                            motor.setPower(0);
                            k=0;
                        }
                        break;
                    case 2:
                        if(shooter.getVelocityErrorUp() >= 0.1 || shooter.getVelocityErrorDown() >= 0.2){
                            motor.setPower(0);
                            k = 1;
                            cycles++;
                        }
                        break;
                }
            }
            telemetry.addData("Target Velocity",targetVelocity);
            telemetry.addData("Velocity Up",shooter.motorUp.getCurrentVelocity());
            telemetry.addData("Velocity Down",shooter.motorDown.getCurrentVelocity());
            telemetry.addData("Power", shooter.motorDown.getPower());
            shooter.update();
            drive.update();
            turret.update();
            telemetry.update();
        }
    }
}