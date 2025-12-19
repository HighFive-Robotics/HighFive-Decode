package org.firstinspires.ftc.teamcode.OpModes.Autos;

import static org.firstinspires.ftc.teamcode.Constants.Globals.autoColor;
import static org.firstinspires.ftc.teamcode.Constants.Globals.finalAutoPose;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Core.Module.Intake.Intake;
import org.firstinspires.ftc.teamcode.Core.Module.Intake.IntakeMotor;
import org.firstinspires.ftc.teamcode.Core.Robot;

@Disabled
@Autonomous(name = "🔴🔝AutoRedClose🔝🔴")
public class AutoRedClose extends LinearOpMode {
    private enum States {
        DriveToPreload,
        ResetForShootPreload,
        ShootSequencePreload,
        DriveToSpike1,
        CollectSpike1,
        DriveToShoot1,
        ResetForShoot1,
        ShootSequence1,
        DriveToSpike2,
        CollectSpike2,
        DriveToShoot2,
        ResetForShoot2,
        ShootSequence2,
        DriveToSpike3,
        CollectSpike3,
        DriveToShoot3,
        ResetForShoot3,
        ShootSequence3,
        DriveToPark,
        DriveToAux, Park
    }

    public Robot robot;
    public States state;
    private int cycles = 0;
    private int shootingState = 0;

    public Pose startPose = new Pose(16, 112, Math.toRadians(0)).mirror();
    private final Pose shootPose = new Pose(45, 102.5, Math.toRadians(-42)).mirror();
    private final Pose shootPosePreload = new Pose(45, 102.5, Math.toRadians(-38)).mirror();
    private final Pose spike1Pose = new Pose(55, 83.5, Math.toRadians(180)).mirror();
    private final Pose collect1Pose = new Pose(22, 83.5, Math.toRadians(180)).mirror();
    private final Pose spike2Pose = new Pose(55, 55, Math.toRadians(180)).mirror();
    private final Pose collect2Pose = new Pose(12, 55, Math.toRadians(180)).mirror();
    private final Pose auxPose = new Pose(24 ,55 , Math.toRadians(180)).mirror();
    private final Pose spike3Pose = new Pose(55, 34, Math.toRadians(180)).mirror();
    private final Pose collect3Pose = new Pose(12, 34, Math.toRadians(180)).mirror();

    private final ElapsedTime opModeTimer = new ElapsedTime();
    private final ElapsedTime stateTimer = new ElapsedTime();
    private final ElapsedTime actionTimer = new ElapsedTime();

    private final double velocity = 3.25;//TODO Verif daca trebuie schimbata
    private final double reverseVelocity = -1;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, startPose, true, Constants.Color.Red, telemetry, gamepad1);
        robot.intake.setCollectType(Intake.CollectTypes.Normal);
        robot.intake.setState(Intake.States.Wait);
        autoColor = Constants.Color.Red;
        robot.drive.resetTeleOpHeading();
        robot.drive.setConstants(Constants.FConstants);

        PathChain preloadPath = robot.drive.pathBuilder()
                .addPath(new BezierLine(startPose, shootPosePreload))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPosePreload.getHeading())
                .build();

        PathChain goForSpike1 = robot.drive.pathBuilder()
                .addPath(new BezierLine(shootPose, spike1Pose))
                .setLinearHeadingInterpolation(shootPosePreload.getHeading(), spike1Pose.getHeading())
                .build();

        PathChain collectSpike1 = robot.drive.pathBuilder()
                .addPath(new BezierLine(spike1Pose, collect1Pose))
                .setLinearHeadingInterpolation(spike1Pose.getHeading(), collect1Pose.getHeading())
                .build();

        PathChain shootFromSpike1 = robot.drive.pathBuilder()
                .addPath(new BezierLine(collect1Pose, shootPose))
                .setLinearHeadingInterpolation(collect1Pose.getHeading(), shootPose.getHeading())
                .build();

        PathChain goForSpike2 = robot.drive.pathBuilder()
                .addPath(new BezierLine(shootPose, spike2Pose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), spike2Pose.getHeading())
                .build();

        PathChain collectSpike2 = robot.drive.pathBuilder()
                .addPath(new BezierLine(spike2Pose, collect2Pose))
                .setLinearHeadingInterpolation(spike2Pose.getHeading(), collect2Pose.getHeading())
                .build();
        PathChain goToAux = robot.drive.pathBuilder()
                .addPath(new BezierLine(collect2Pose , auxPose))
                .setConstantHeadingInterpolation(collect2Pose.getHeading())
                .build();
        PathChain shootFromSpike2 = robot.drive.pathBuilder()
                .addPath(new BezierLine(auxPose, shootPose))
                .setLinearHeadingInterpolation(auxPose.getHeading(), shootPose.getHeading())
                .build();


        PathChain goForSpike3 = robot.drive.pathBuilder()
                .addPath(new BezierLine(shootPose, spike3Pose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), spike3Pose.getHeading())
                .build();

        PathChain collectSpike3 = robot.drive.pathBuilder()
                .addPath(new BezierLine(spike3Pose, collect3Pose))
                .setLinearHeadingInterpolation(spike3Pose.getHeading(), collect3Pose.getHeading())
                .build();

        PathChain shootFromSpike3 = robot.drive.pathBuilder()
                .addPath(new BezierLine(collect3Pose, shootPose))
                .setLinearHeadingInterpolation(collect3Pose.getHeading(), shootPose.getHeading())
                .build();

        PathChain goToPark = robot.drive.pathBuilder()
                .addPath(new BezierLine(shootPose, collect1Pose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), collect1Pose.getHeading())
                .build();

        telemetry.addLine("Ready");
        telemetry.update();
        waitForStart();
        opModeTimer.reset();
        stateTimer.reset();
        actionTimer.reset();
        state = AutoRedClose.States.DriveToPreload;
        while (opModeIsActive()) {
            switch (state) {
                case DriveToPreload:
                    robot.drive.setMaxPower(0.8);
                    robot.drive.followPath(preloadPath, true);
                    robot.shooter.setTargetVelocity(velocity);
                    stateTimer.reset();
                    state = AutoRedClose.States.ResetForShootPreload;
                    break;
                case ResetForShootPreload:
                    if (robot.isDone() || stateTimer.milliseconds() > 5000) {
                        robot.drive.setMaxPower(1);
                        robot.intake.setPower(IntakeMotor.States.Wait);
                        stateTimer.reset();
                        actionTimer.reset();
                        cycles = 0;
                        shootingState = 0;
                        state = AutoRedClose.States.ShootSequencePreload;
                    }
                    break;
                case ShootSequencePreload:
                    if (runShootingSequence()) {
                        robot.drive.setMaxPower(1);
                        robot.intake.setPower(IntakeMotor.States.Collect);
                        stateTimer.reset();
                        state = AutoRedClose.States.DriveToSpike1;
                    }
                    break;
                case DriveToSpike1:
                    if (stateTimer.milliseconds() >= 300) {
                        robot.drive.followPath(goForSpike1, true);
                        robot.shooter.setTargetVelocity(reverseVelocity);
                        stateTimer.reset();
                        state = AutoRedClose.States.CollectSpike1;
                    }
                    break;
                case CollectSpike1:
                    if (robot.isDone() || stateTimer.milliseconds() > 5000) {
                        robot.drive.setMaxPower(0.5);
                        robot.drive.followPath(collectSpike1, true);
                        stateTimer.reset();
                        state = AutoRedClose.States.DriveToShoot1;
                    }
                    break;
                case DriveToShoot1:
                    if (robot.isDone() || stateTimer.milliseconds() > 5000) {
                        robot.drive.setMaxPower(1);
                        robot.drive.followPath(shootFromSpike1, true);
                        stateTimer.reset();
                        state = AutoRedClose.States.ResetForShoot1;
                    }
                    break;
                case ResetForShoot1:
                    if (robot.isDone() || stateTimer.milliseconds() > 5000) {
                        robot.intake.intakeMotor.setPower(-0.7);
                        stateTimer.reset();
                        actionTimer.reset();
                        cycles = 0;
                        shootingState = -1;
                        state = AutoRedClose.States.ShootSequence1;
                    }
                    break;
                case ShootSequence1:
                    if (runShootingSequence()) {
                        robot.drive.setMaxPower(1);
                        robot.intake.setPower(IntakeMotor.States.Wait);
                        stateTimer.reset();
                        state = AutoRedClose.States.DriveToSpike2;
                    }
                    break;
                case DriveToSpike2:
                    if (stateTimer.milliseconds() >= 300) {
                        robot.drive.followPath(goForSpike2, true);
                        robot.shooter.setTargetVelocity(reverseVelocity);
                        robot.intake.setPower(IntakeMotor.States.Collect);
                        stateTimer.reset();
                        state = AutoRedClose.States.CollectSpike2;
                    }
                    break;
                case CollectSpike2:
                    if (robot.isDone() || stateTimer.milliseconds() > 5000) {
                        robot.drive.setMaxPower(0.5);
                        robot.drive.followPath(collectSpike2, true);
                        stateTimer.reset();
                        state = AutoRedClose.States.DriveToAux;
                    }
                    break;
                case DriveToAux:
                    if(robot.isDone()){
                        robot.drive.setMaxPower(1);
                        robot.drive.followPath(goToAux, false);
                        state = AutoRedClose.States.DriveToShoot2;
                    }
                case DriveToShoot2:
                    if (robot.isDone() || stateTimer.milliseconds() > 5000) {
                        robot.drive.followPath(shootFromSpike2, true);
                        stateTimer.reset();
                        state = AutoRedClose.States.ResetForShoot2;
                    }
                    break;
                case ResetForShoot2:
                    if (robot.isDone() || stateTimer.milliseconds() > 5000) {
                        robot.intake.intakeMotor.setPower(-0.7);
                        stateTimer.reset();
                        actionTimer.reset();
                        cycles = 0;
                        shootingState = -1;
                        state = AutoRedClose.States.ShootSequence2;
                    }
                    break;
                case ShootSequence2:
                    if (runShootingSequence()) {
                        robot.drive.setMaxPower(1);
                        robot.intake.setPower(IntakeMotor.States.Collect);
                        stateTimer.reset();
                        state = AutoRedClose.States.DriveToSpike3;
                    }
                    break;
                case DriveToSpike3:
                    if (stateTimer.milliseconds() >= 300) {
                        robot.drive.followPath(goForSpike3, true);
                        robot.shooter.setTargetVelocity(reverseVelocity);
                        stateTimer.reset();
                        state = AutoRedClose.States.CollectSpike3;
                    }
                    break;
                case CollectSpike3:
                    if (robot.isDone() || stateTimer.milliseconds() > 5000) {
                        robot.drive.setMaxPower(0.5);
                        robot.drive.followPath(collectSpike3, true);
                        stateTimer.reset();
                        state = AutoRedClose.States.DriveToShoot3;
                    }
                    break;
                case DriveToShoot3:
                    if (robot.isDone() || stateTimer.milliseconds() > 5000) {
                        robot.drive.setMaxPower(1);
                        robot.drive.followPath(shootFromSpike3, true);
                        stateTimer.reset();
                        state = AutoRedClose.States.ResetForShoot3;
                    }
                    break;
                case ResetForShoot3:
                    if (robot.isDone() || stateTimer.milliseconds() > 5000) {
                        robot.intake.intakeMotor.setPower(-0.7);
                        stateTimer.reset();
                        actionTimer.reset();
                        cycles = 0;
                        shootingState = -1;
                        state = AutoRedClose.States.ShootSequence3;
                    }
                    break;
                case ShootSequence3:
                    if (runShootingSequence()) {
                        robot.drive.setMaxPower(1);
                        stateTimer.reset();
                        state = AutoRedClose.States.DriveToPark;
                    }
                    break;
                case DriveToPark:
                    if (stateTimer.milliseconds() >= 100) {
                        robot.drive.followPath(goToPark, true);
                        robot.shooter.setTargetVelocity(0);
                        stateTimer.reset();
                        state = AutoRedClose.States.Park;
                    }
                    break;
                case Park:
                    if (robot.isDone() || stateTimer.milliseconds() > 4000) {
                        robot.drive.breakFollowing();
                        finalAutoPose = robot.drive.getPose();
                    }
                    break;
            }
            if (opModeTimer.milliseconds() > 27000 && state != AutoRedClose.States.DriveToPark && state != AutoRedClose.States.Park) {
                state = AutoRedClose.States.DriveToPark;
                stateTimer.reset();
            }
            telemetry.addData("State", state);
            telemetry.addData("Cycles", cycles);
            telemetry.addData("Shooter Velocity", robot.shooter.motorUp.getCurrentVelocity());
            telemetry.addData("Robot Pose", robot.drive.getPose());
            telemetry.update();
            robot.update();
        }
    }

    public boolean runShootingSequence() {
        switch (shootingState) {
            case -1:
                if(actionTimer.milliseconds()>=250){
                    robot.shooter.setTargetVelocity(velocity);
                    actionTimer.reset();
                    shootingState=0;
                }
                break;
            case 0:
                if (robot.shooter.atTarget() && actionTimer.milliseconds() > 200) {
                    robot.intake.setPower(IntakeMotor.States.Collect);
                    actionTimer.reset();
                    shootingState = 1;
                }
                break;
            case 1:
                if (actionTimer.milliseconds() > 1200 || robot.shooter.getVelocityError() >= 0.7) {
                    robot.intake.setPower(IntakeMotor.States.Wait);
                    actionTimer.reset();
                    stateTimer.reset();
                    shootingState = 0;
                    cycles++;
                    if (cycles >= 3) {
                        return true;
                    }
                }
                break;
        }
        return false;
    }
}