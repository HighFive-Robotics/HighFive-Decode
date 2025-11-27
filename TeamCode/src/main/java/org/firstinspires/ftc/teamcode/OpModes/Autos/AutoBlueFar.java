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

@Autonomous(name = "🔵🔝AutoBlueFar🔝🔵")
public class AutoBlueFar extends LinearOpMode {
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

    public Pose startPose = new Pose(65, 12, Math.toRadians(-90));
    private final Pose shootPose = new Pose(62, 22, Math.toRadians(-63.5));

    private final Pose collect1Pose = new Pose(20, 40, Math.toRadians(180));

    private final ElapsedTime opModeTimer = new ElapsedTime();
    private final ElapsedTime stateTimer = new ElapsedTime();
    private final ElapsedTime actionTimer = new ElapsedTime();

    private final double velocity = 5.5;
    private final double reverseVelocity = -1;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, startPose, true, Constants.Color.Blue, telemetry, gamepad1);
        PathChain preloadPath = robot.drive.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
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
                    robot.drive.setMaxPower(0.5);
                    robot.drive.followPath(preloadPath, true);
                    robot.shooter.setTargetVelocity(velocity);
                    stateTimer.reset();
                    state = States.ResetForShootPreload;
                    break;
                case ResetForShootPreload:
                    if (robot.isDone() || stateTimer.milliseconds() > 5000) {
                        robot.drive.setMaxPower(1);
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