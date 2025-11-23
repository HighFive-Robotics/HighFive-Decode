package org.firstinspires.ftc.teamcode.OpModes.Autos;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Core.Module.Intake.Intake;
import org.firstinspires.ftc.teamcode.Core.Robot;

@Autonomous(name = "AutoRedClose BAMBAM")
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
        Park
    }

    public Robot robot;
    public States state;
    private int cycles = 0;
    private int shootingState = 0;

    public Pose startPose = new Pose(16, 112, Math.toRadians(0));
    private final Pose shootPose = new Pose(45, 102.5, Math.toRadians(-42));
    private final Pose spike1Pose = new Pose(55, 85, Math.toRadians(180));
    private final Pose collect1Pose = new Pose(22, 85, Math.toRadians(180));
    private final Pose spike2Pose = new Pose(55, 58, Math.toRadians(180));
    private final Pose collect2Pose = new Pose(12, 58, Math.toRadians(180));
    private final Pose spike3Pose = new Pose(55, 35, Math.toRadians(180));
    private final Pose collect3Pose = new Pose(12, 35, Math.toRadians(180));

    private final ElapsedTime opModeTimer = new ElapsedTime();
    private final ElapsedTime stateTimer = new ElapsedTime();
    private final ElapsedTime actionTimer = new ElapsedTime();

    private final double velocity = 3.4;
    private final double reverseVelocity = -1;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, startPose, true, Constants.Color.Blue, telemetry, gamepad1);
        PathChain preloadPath = robot.drive.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        PathChain goForSpike1 = robot.drive.pathBuilder()
                .addPath(new BezierLine(shootPose, spike1Pose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), spike1Pose.getHeading())
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

        PathChain shootFromSpike2 = robot.drive.pathBuilder()
                .addPath(new BezierLine(collect2Pose, shootPose))
                .setLinearHeadingInterpolation(collect2Pose.getHeading(), shootPose.getHeading())
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
        state = States.DriveToPreload;
        while (opModeIsActive()) {
            switch (state) {
                case DriveToPreload:
                    robot.drive.followPath(preloadPath, true);
                    robot.shooter.setTargetVelocity(velocity);
                    stateTimer.reset();
                    state = States.ResetForShootPreload;
                    break;
                case ResetForShootPreload:
                    if (robot.isDone() || stateTimer.milliseconds() > 5000) {
                        robot.intake.setAction(Intake.IntakeActions.Wait);
                        stateTimer.reset();
                        actionTimer.reset();
                        cycles = 0;
                        shootingState = 0;
                        state = States.ShootSequencePreload;
                    }
                    break;
                case ShootSequencePreload:
                    if (runShootingSequence()) {
                        robot.drive.setMaxPower(1);
                        robot.intake.setAction(Intake.IntakeActions.Collect);
                        stateTimer.reset();
                        state = States.DriveToSpike1;
                    }
                    break;
                case DriveToSpike1:
                    if (stateTimer.milliseconds() >= 300) {
                        robot.drive.followPath(goForSpike1, true);
                        robot.shooter.setTargetVelocity(reverseVelocity);
                        stateTimer.reset();
                        state = States.CollectSpike1;
                    }
                    break;
                case CollectSpike1:
                    if (robot.isDone() || stateTimer.milliseconds() > 5000) {
                        robot.drive.setMaxPower(0.5);
                        robot.drive.followPath(collectSpike1, true);
                        stateTimer.reset();
                        state = States.DriveToShoot1;
                    }
                    break;
                case DriveToShoot1:
                    if (robot.isDone() || stateTimer.milliseconds() > 5000) {
                        robot.drive.setMaxPower(1);
                        robot.drive.followPath(shootFromSpike1, true);
                        stateTimer.reset();
                        state = States.ResetForShoot1;
                    }
                    break;
                case ResetForShoot1:
                    if (robot.isDone() || stateTimer.milliseconds() > 5000) {
                        robot.intake.motorIntake.setPower(-0.7);
                        stateTimer.reset();
                        actionTimer.reset();
                        cycles = 0;
                        shootingState = -1;
                        state = States.ShootSequence1;
                    }
                    break;
                case ShootSequence1:
//                    if(cycles == 0 ){
//                        robot.drive.deactivateAllPIDFs();
//                        robot.drive.activateTranslational();
//                        robot.drive.update();
//                        robot.drive.updateDrivetrain();
//                    }
                    if (runShootingSequence()) {
                        robot.drive.setMaxPower(1);
                        robot.intake.setAction(Intake.IntakeActions.Wait);
                        stateTimer.reset();
                        state = States.DriveToSpike2;
                    }
                    break;
                case DriveToSpike2:
                    if (stateTimer.milliseconds() >= 300) {
                        robot.drive.followPath(goForSpike2, true);
                        robot.shooter.setTargetVelocity(reverseVelocity);
                        robot.intake.setAction(Intake.IntakeActions.Collect);
                        stateTimer.reset();
                        state = States.CollectSpike2;
                    }
                    break;
                case CollectSpike2:
                    if (robot.isDone() || stateTimer.milliseconds() > 5000) {
                        robot.drive.setMaxPower(0.5);
                        robot.drive.followPath(collectSpike2, true);
                        stateTimer.reset();
                        state = States.DriveToShoot2;
                    }
                    break;
                case DriveToShoot2:
                    if (robot.isDone() || stateTimer.milliseconds() > 5000) {
                        robot.drive.setMaxPower(1);
                        robot.drive.followPath(shootFromSpike2, true);
                        stateTimer.reset();
                        state = States.ResetForShoot2;
                    }
                    break;
                case ResetForShoot2:
                    if (robot.isDone() || stateTimer.milliseconds() > 5000) {
                        robot.intake.motorIntake.setPower(-0.7);
                        stateTimer.reset();
                        actionTimer.reset();
                        cycles = 0;
                        shootingState = -1;
                        state = States.ShootSequence2;
                    }
                    break;
                case ShootSequence2:
                    if (runShootingSequence()) {
                        robot.drive.setMaxPower(1);
                        robot.intake.setAction(Intake.IntakeActions.Collect);
                        stateTimer.reset();
                        state = States.DriveToSpike3;
                    }
                    break;
                case DriveToSpike3:
                    if (stateTimer.milliseconds() >= 300) {
                        robot.drive.followPath(goForSpike3, true);
                        robot.shooter.setTargetVelocity(reverseVelocity);
                        stateTimer.reset();
                        state = States.CollectSpike3;
                    }
                    break;
                case CollectSpike3:
                    if (robot.isDone() || stateTimer.milliseconds() > 5000) {
                        robot.drive.setMaxPower(0.5);
                        robot.drive.followPath(collectSpike3, true);
                        stateTimer.reset();
                        state = States.DriveToShoot3;
                    }
                    break;
                case DriveToShoot3:
                    if (robot.isDone() || stateTimer.milliseconds() > 5000) {
                        robot.drive.setMaxPower(1);
                        robot.drive.followPath(shootFromSpike3, true);
                        stateTimer.reset();
                        state = States.ResetForShoot3;
                    }
                    break;
                case ResetForShoot3:
                    if (robot.isDone() || stateTimer.milliseconds() > 5000) {
                        robot.intake.motorIntake.setPower(-0.7);
                        stateTimer.reset();
                        actionTimer.reset();
                        cycles = 0;
                        shootingState = -1;
                        state = States.ShootSequence3;
                    }
                    break;
                case ShootSequence3:
                    if (runShootingSequence()) {
                        robot.drive.setMaxPower(1);
                        stateTimer.reset();
                        state = States.DriveToPark;
                    }
                    break;
                case DriveToPark:
                    if (stateTimer.milliseconds() >= 100) {
                        robot.drive.followPath(goToPark, true);
                        robot.shooter.setTargetVelocity(0);
                        stateTimer.reset();
                        state = States.Park;
                    }
                    break;
                case Park:
                    if (robot.isDone() || stateTimer.milliseconds() > 4000) {
                        robot.drive.breakFollowing();
                    }
                    break;
            }
            if (opModeTimer.milliseconds() > 27000 && state != States.DriveToPark && state != States.Park) {
                state = States.DriveToPark;
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
                    robot.intake.setAction(Intake.IntakeActions.Collect);
                    actionTimer.reset();
                    shootingState = 1;
                }
                break;
            case 1:
                if (actionTimer.milliseconds() > 1200 || robot.shooter.getVelocityError() >= 0.7) {
                    robot.intake.setAction(Intake.IntakeActions.Wait);
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