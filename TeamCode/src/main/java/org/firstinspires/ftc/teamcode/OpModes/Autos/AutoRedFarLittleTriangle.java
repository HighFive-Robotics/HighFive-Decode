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
import org.firstinspires.ftc.teamcode.Core.Robot;

@Autonomous(name = "🔴🔝AutoRedFar🔝🔴")
public class AutoRedFarLittleTriangle extends LinearOpMode {
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
        CollectArtefacts,
        DriveToAux, Park
    }

    public Robot robot;

    public States state;
    private int cycles = 0;
    private int shootingState = 0;

    public Pose startPose = new Pose(58, 0, Math.toRadians(-90)).mirror();
    private final Pose shootPose = new Pose(54, 8, Math.toRadians(-67)).mirror();
    private final Pose preCollect1Pose = new Pose(27.5, 0, Math.toRadians(-90)).mirror();

    private final ElapsedTime opModeTimer = new ElapsedTime();
    private final ElapsedTime stateTimer = new ElapsedTime();
    private final ElapsedTime actionTimer = new ElapsedTime();

    private final double velocity = 4.8; // TODO Verif daca trebuie schimbata
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
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();
        PathChain goToCollect = robot.drive.pathBuilder()
                .addPath(new BezierLine(shootPose, preCollect1Pose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), preCollect1Pose.getHeading())
                .build();

        telemetry.addLine("Ready");
        telemetry.update();
        waitForStart();
        robot.shooter.setTargetVelocity(reverseVelocity);
        opModeTimer.reset();
        stateTimer.reset();
        actionTimer.reset();
        state = AutoRedFarLittleTriangle.States.DriveToPreload;
        while (opModeIsActive()) {
            switch (state) {
                case DriveToPreload:
                    if (stateTimer.milliseconds() >= 300) {
                        robot.drive.followPath(preloadPath, true);
                        robot.shooter.setTargetVelocity(reverseVelocity);
                        stateTimer.reset();
                        state = AutoRedFarLittleTriangle.States.ResetForShootPreload;
                        robot.intake.setPower(IntakeMotor.States.Collect);
                        actionTimer.reset();
                    }
                    break;
                case ResetForShootPreload:
                    if (robot.isDone()) {
                        robot.intake.intakeMotor.setPower(-0.7);
                        stateTimer.reset();
                        actionTimer.reset();
                        cycles = 0;
                        shootingState = -1;
                        state = States.ShootSequencePreload;
                    }
                    break;
                case ShootSequencePreload:
                    if (runShootingSequence()) {
                        robot.drive.setMaxPower(1);
                        robot.intake.setPower(IntakeMotor.States.Wait);
                        stateTimer.reset();
                        state = States.CollectArtefacts;
                    }
                    break;
                case CollectArtefacts:
                    if (stateTimer.milliseconds() >= 100) {
                        robot.drive.followPath(goToCollect, true);
                        robot.shooter.setTargetVelocity(0);
                        stateTimer.reset();
                        state = AutoRedFarLittleTriangle.States.Park;
                    }
                    break;
                case Park:
                    if (robot.isDone() || stateTimer.milliseconds() > 4000) {
                        robot.drive.breakFollowing();
                        finalAutoPose = robot.drive.getPose();
                    }
                    break;
            }
            if (opModeTimer.milliseconds() > 27000 && state != AutoRedFarLittleTriangle.States.CollectArtefacts && state != AutoRedFarLittleTriangle.States.Park) {
                state = AutoRedFarLittleTriangle.States.CollectArtefacts;
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
                if (actionTimer.milliseconds() >= 185) {
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