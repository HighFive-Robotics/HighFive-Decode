package org.firstinspires.ftc.teamcode.OpModes.Autos;

import static org.firstinspires.ftc.teamcode.Constants.Globals.autoColor;
import static org.firstinspires.ftc.teamcode.Constants.Globals.finalAutoPose;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Core.Module.Intake.Intake;
import org.firstinspires.ftc.teamcode.Core.Module.Intake.IntakeMotor;
import org.firstinspires.ftc.teamcode.Core.Module.Outtake.BlockerOuttake;
import org.firstinspires.ftc.teamcode.Core.Robot;

@Autonomous(name = "🔴🔝AutoRedCloseLm2🔝🔴")
public class AutoRedCloseLm2 extends LinearOpMode {
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
    private final Pose shootPose1 = new Pose(44, 102.5, Math.toRadians(-42)).mirror();
    private final Pose shootPose2 = new Pose(44, 102.5, Math.toRadians(-42)).mirror();
    private final Pose shootPose3 = new Pose(44, 102.5, Math.toRadians(-42)).mirror();
    private final Pose shootPosePreload = new Pose(44, 102.5, Math.toRadians(-42)).mirror();
    private final Pose spike1Pose = new Pose(52, 83.5, Math.toRadians(180)).mirror();
    private final Pose collect1Pose = new Pose(22, 83.5, Math.toRadians(180)).mirror();
    private final Pose spike2Pose = new Pose(52, 57, Math.toRadians(180)).mirror();
    private final Pose collect2Pose = new Pose(12, 57, Math.toRadians(180)).mirror();
    private final Pose auxPose = new Pose(23, 54, Math.toRadians(180)).mirror();
    private final Pose spike3Pose = new Pose(52, 35, Math.toRadians(180)).mirror();
    private final Pose collect3Pose = new Pose(12, 35, Math.toRadians(180)).mirror();
    private final Pose finalPose = new Pose(15, 83.5, Math.toRadians(180)).mirror();



    private final ElapsedTime opModeTimer = new ElapsedTime();
    private final ElapsedTime stateTimer = new ElapsedTime();
    private final ElapsedTime actionTimer = new ElapsedTime();
    private final double velocity = 3.3;
    public boolean justStarted = true;
    private final double reverseVelocity = -1.35;

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
                .addPath(new BezierLine(shootPose1, spike1Pose))
                .setLinearHeadingInterpolation(shootPosePreload.getHeading(), spike1Pose.getHeading())
                .build();

        PathChain collectSpike1 = robot.drive.pathBuilder()
                .addPath(new BezierLine(spike1Pose, collect1Pose))
                .setLinearHeadingInterpolation(spike1Pose.getHeading(), collect1Pose.getHeading())
                .build();

        PathChain shootFromSpike1 = robot.drive.pathBuilder()
                .addPath(new BezierLine(collect1Pose, shootPose1))
                .setLinearHeadingInterpolation(collect1Pose.getHeading(), shootPose1.getHeading())
                .build();

        PathChain goForSpike2 = robot.drive.pathBuilder()
                .addPath(new BezierLine(shootPose1, spike2Pose))
                .setLinearHeadingInterpolation(shootPose1.getHeading(), spike2Pose.getHeading())
                .build();

        PathChain collectSpike2 = robot.drive.pathBuilder()
                .addPath(new BezierLine(spike2Pose, collect2Pose))
                .setLinearHeadingInterpolation(spike2Pose.getHeading(), collect2Pose.getHeading())
                .build();
        PathChain goToAux = robot.drive.pathBuilder()
                .addPath(new BezierLine(collect2Pose, auxPose))
                .setConstantHeadingInterpolation(collect2Pose.getHeading())
                .build();
        PathChain shootFromSpike2 = robot.drive.pathBuilder()
                .addPath(new BezierLine(auxPose, shootPose2))
                .setLinearHeadingInterpolation(auxPose.getHeading(), shootPose2.getHeading())
                .build();


        PathChain goForSpike3 = robot.drive.pathBuilder()
                .addPath(new BezierLine(shootPose2, spike3Pose))
                .setLinearHeadingInterpolation(shootPose2.getHeading(), spike3Pose.getHeading())
                .build();

        PathChain collectSpike3 = robot.drive.pathBuilder()
                .addPath(new BezierLine(spike3Pose, collect3Pose))
                .setLinearHeadingInterpolation(spike3Pose.getHeading(), collect3Pose.getHeading())
                .build();

        PathChain shootFromSpike3 = robot.drive.pathBuilder()
                .addPath(new BezierLine(collect3Pose, shootPose3))
                .setLinearHeadingInterpolation(collect3Pose.getHeading(), shootPose3.getHeading())
                .build();

        PathChain goToPark = robot.drive.pathBuilder()
                .addPath(new BezierLine(shootPose3, finalPose))
                .setLinearHeadingInterpolation(shootPose3.getHeading(), collect1Pose.getHeading())
                .build();

        telemetry.addLine("Ready");
        telemetry.update();
        waitForStart();
        opModeTimer.reset();
        stateTimer.reset();
        actionTimer.reset();
        state = AutoRedCloseLm2.States.DriveToPreload;
        while (opModeIsActive()) {
            if (justStarted) {
                opModeTimer.reset();
                justStarted = false;
                robot.shooter.blocker.setState(BlockerOuttake.States.Open);
                robot.drive.turnTo(0);
            }
            switch (state) {
                case DriveToPreload:
                    /*robot.drive.setMaxPower(0.93);
                    robot.drive.followPath(preloadPath, true);
                    robot.shooter.setTargetVelocity(velocity);
                    stateTimer.reset();
                    state = AutoRedCloseLm2.States.ResetForShootPreload;*/
                    break;
                case ResetForShootPreload:
                    if (robot.isDone() || stateTimer.milliseconds() > 5000) {
                        robot.drive.setMaxPower(1);
                        robot.intake.setPower(IntakeMotor.States.Wait);
                        stateTimer.reset();
                        actionTimer.reset();
                        cycles = 0;
                        shootingState = 0;
                        state = AutoRedCloseLm2.States.ShootSequencePreload;
                    }
                    break;
                case ShootSequencePreload:
                    if (runShootingSequence()) {
                        robot.drive.setMaxPower(1);
                        robot.intake.setPower(IntakeMotor.States.Collect);
                        robot.shooter.setTargetVelocity(reverseVelocity);
                        stateTimer.reset();
                        state = AutoRedCloseLm2.States.DriveToSpike1;
                    }
                    break;
                case DriveToSpike1:
                    if (stateTimer.milliseconds() >= 300) {
                        robot.drive.followPath(goForSpike1, true);
                        stateTimer.reset();
                        state = AutoRedCloseLm2.States.CollectSpike1;
                    }
                    break;
                case CollectSpike1:
                    if (robot.isDone() || stateTimer.milliseconds() > 5000) {
                        robot.drive.setMaxPower(0.65);
                        robot.drive.followPath(collectSpike1, true);
                        stateTimer.reset();
                        state = AutoRedCloseLm2.States.DriveToShoot1;
                    }
                    break;
                case DriveToShoot1:
                    if (robot.isDone() || stateTimer.milliseconds() > 5000) {
                        robot.drive.setMaxPower(1);
                        robot.drive.followPath(shootFromSpike1, true);
                        stateTimer.reset();
                        state = AutoRedCloseLm2.States.ResetForShoot1;
                    }
                    break;
                case ResetForShoot1:
                    if (robot.isDone() || stateTimer.milliseconds() > 5000) {
                        robot.intake.intakeMotor.setPower(-0.7);
                        stateTimer.reset();
                        actionTimer.reset();
                        cycles = 0;
                        shootingState = -1;
                        state = AutoRedCloseLm2.States.ShootSequence1;
                    }
                    break;
                case ShootSequence1:
                    if (runShootingSequence()) {
                        robot.drive.setMaxPower(1);
                        robot.intake.setPower(IntakeMotor.States.Wait);
                        stateTimer.reset();
                        state = AutoRedCloseLm2.States.DriveToSpike2;
                    }
                    break;
                case DriveToSpike2:
                    if (stateTimer.milliseconds() >= 300) {
                        robot.drive.followPath(goForSpike2, true);
                        robot.shooter.setTargetVelocity(reverseVelocity);
                        robot.intake.setPower(IntakeMotor.States.Collect);
                        stateTimer.reset();
                        state = AutoRedCloseLm2.States.CollectSpike2;
                    }
                    break;
                case CollectSpike2:
                    if (robot.isDone() || stateTimer.milliseconds() > 5000) {
                        robot.drive.setMaxPower(0.72);
                        robot.drive.followPath(collectSpike2, true);
                        stateTimer.reset();
                        state = AutoRedCloseLm2.States.DriveToAux;
                    }
                    break;
                case DriveToAux:
                    if (robot.isDone()) {
                        robot.drive.setMaxPower(1);
                        robot.drive.followPath(goToAux, false);
                        state = AutoRedCloseLm2.States.DriveToShoot2;
                    }
                case DriveToShoot2:
                    if (robot.isDone() || stateTimer.milliseconds() > 5000) {
                        robot.drive.followPath(shootFromSpike2, true);
                        stateTimer.reset();
                        state = AutoRedCloseLm2.States.ResetForShoot2;
                    }
                    break;
                case ResetForShoot2:
                    if (robot.isDone() || stateTimer.milliseconds() > 5000) {
                        robot.intake.intakeMotor.setPower(-0.7);
                        stateTimer.reset();
                        actionTimer.reset();
                        cycles = 0;
                        shootingState = -1;
                        state = AutoRedCloseLm2.States.ShootSequence2;
                    }
                    break;
                case ShootSequence2:
                    if (runShootingSequence()) {
                        robot.drive.setMaxPower(1);
                        robot.intake.setPower(IntakeMotor.States.Collect);
                        stateTimer.reset();
                        state = AutoRedCloseLm2.States.DriveToSpike3;
                    }
                    break;
                case DriveToSpike3:
                    if (stateTimer.milliseconds() >= 300) {
                        robot.drive.followPath(goForSpike3, true);
                        robot.shooter.setTargetVelocity(reverseVelocity);
                        stateTimer.reset();
                        state = AutoRedCloseLm2.States.CollectSpike3;
                    }
                    break;
                case CollectSpike3:
                    if (robot.isDone() || stateTimer.milliseconds() > 5000) {
                        robot.drive.setMaxPower(0.72);
                        robot.drive.followPath(collectSpike3, true);
                        stateTimer.reset();
                        state = AutoRedCloseLm2.States.DriveToShoot3;
                    }
                    break;
                case DriveToShoot3:
                    if (robot.isDone() || stateTimer.milliseconds() > 5000) {
                        robot.drive.setMaxPower(1);
                        robot.drive.followPath(shootFromSpike3, true);
                        stateTimer.reset();
                        state = AutoRedCloseLm2.States.ResetForShoot3;
                    }
                    break;
                case ResetForShoot3:
                    if (robot.isDone() || stateTimer.milliseconds() > 5000) {
                        robot.intake.intakeMotor.setPower(-0.7);
                        stateTimer.reset();
                        actionTimer.reset();
                        cycles = 0;
                        shootingState = -1;
                        state = AutoRedCloseLm2.States.ShootSequence3;
                    }
                    break;
                case ShootSequence3:
                    if (runShootingSequence()) {
                        robot.drive.setMaxPower(1);
                        stateTimer.reset();
                        state = AutoRedCloseLm2.States.DriveToPark;
                    }
                    break;
                case DriveToPark:
                    if (stateTimer.milliseconds() >= 100) {
                        robot.drive.followPath(goToPark, true);
                        robot.shooter.setTargetVelocity(0);
                        stateTimer.reset();
                        state = AutoRedCloseLm2.States.Park;
                        finalAutoPose = robot.drive.getPose();
                    }
                    break;
                case Park:
                    if (robot.isDone() || stateTimer.milliseconds() > 4000) {
                        robot.drive.breakFollowing();
                        finalAutoPose = robot.drive.getPose();
                    }
                    break;
            }
            if (opModeTimer.milliseconds() > 27000 && state != AutoRedCloseLm2.States.DriveToPark && state != AutoRedCloseLm2.States.Park) {
                state = AutoRedCloseLm2.States.DriveToPark;
                stateTimer.reset();
            }
            finalAutoPose = robot.drive.getPose();
            telemetry.addData("State", state);
            telemetry.addData("Cycles", cycles);
            telemetry.addData("Shooter Velocity", robot.shooter.motorUp.getCurrentVelocity());
            telemetry.addData("Robot Pose", robot.drive.getPose());
            telemetry.addData("Final Robot Auto Pose", finalAutoPose);
            telemetry.update();
            robot.update();
        }
    }

    public boolean runShootingSequence() {
        switch (shootingState) {
            case -1:
                if (actionTimer.milliseconds() >= 215) {
                    robot.shooter.setTargetVelocity(velocity);
                    robot.intake.intakeMotor.setPower(0);
                    actionTimer.reset();
                    shootingState = 0;
                }
                break;
            case 0:
                if (robot.shooter.atTarget() && actionTimer.milliseconds() >= 210) {
                    robot.intake.intakeMotor.setPower(0.66);
                    actionTimer.reset();
                    stateTimer.reset();
                    shootingState = 1;
                }
                break;
            case 1:
                if (actionTimer.milliseconds() > 800 || (robot.shooter.getVelocityError() >= 0.7 && stateTimer.milliseconds() >= 150)) {
                    robot.shooter.setTargetVelocity(velocity);
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