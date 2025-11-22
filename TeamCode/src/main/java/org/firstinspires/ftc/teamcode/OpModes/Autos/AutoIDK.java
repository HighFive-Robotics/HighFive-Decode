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

@Autonomous(name = "IDK NO WORK")
public class AutoIDK extends LinearOpMode {
    public Robot robot;
    private int state = 0;

    public Pose startPose = new Pose(16, 112, Math.toRadians(0));
    private final Pose shootPose = new Pose(45, 102.5, Math.toRadians(-40));
    private final Pose spike1Pose = new Pose(55, 85, Math.toRadians(180));
    private final Pose collect1Pose = new Pose(22, 85, Math.toRadians(180));
    private final Pose spike2Pose = new Pose(55, 58, Math.toRadians(180));
    private final Pose collect2Pose = new Pose(16, 58, Math.toRadians(180));
    private final Pose spike3Pose = new Pose(55, 35, Math.toRadians(180));
    private final Pose collect3Pose = new Pose(16, 35, Math.toRadians(180));

    private final ElapsedTime pathTimer = new ElapsedTime();
    private final ElapsedTime actionTimer = new ElapsedTime();
    private final ElapsedTime opModeTimer = new ElapsedTime();

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
        pathTimer.reset();
        actionTimer.reset();

        while (opModeIsActive()) {
            robot.update();

            switch (state) {
                case 0:
                    robot.drive.followPath(preloadPath, true);
                    robot.shooter.setTargetVelocity(3.5);
                    pathTimer.reset();
                    state = 1;
                    break;

                case 1:
                    if (robot.isDone()|| pathTimer.seconds() > 3.0) {
                        robot.intake.setAction(Intake.IntakeActions.Wait);
                        actionTimer.reset();
                        state = 2;
                    }
                    break;

                case 2:
                    if (robot.shooter.atTarget() && actionTimer.milliseconds() >= 250) {
                        robot.intake.setAction(Intake.IntakeActions.Collect);
                        actionTimer.reset();
                        state = 3;
                    } else if (actionTimer.seconds() > 1.5) {
                        robot.intake.setAction(Intake.IntakeActions.Collect);
                        actionTimer.reset();
                        state = 3;
                    }
                    break;

                case 3:
                    if (actionTimer.milliseconds() >= 300) {
                        robot.intake.setAction(Intake.IntakeActions.Collect);
                        robot.drive.setMaxPower(1);
                        robot.drive.followPath(goForSpike1, true);
                        robot.shooter.setTargetVelocity(-1);
                        pathTimer.reset();
                        state = 4;
                    }
                    break;

                case 4:
                    if ((robot.isDone()|| pathTimer.seconds() > 3.0) && actionTimer.milliseconds() >= 250) {
                        robot.drive.followPath(collectSpike1, true);
                        robot.drive.setMaxPower(0.5);
                        pathTimer.reset();
                        state = 5;
                    }
                    break;

                case 5:
                    if (robot.isDone()|| pathTimer.seconds() > 3.0) {
                        robot.drive.setMaxPower(1);
                        robot.drive.followPath(shootFromSpike1, true);
                        pathTimer.reset();
                        state = 6;
                    }
                    break;

                case 6:
                    if (robot.isDone()|| pathTimer.seconds() > 3.0) {
                        robot.intake.motorIntake.setPower(-0.7);
                        actionTimer.reset();
                        state = 7;
                    }
                    break;

                case 7:
                    if (actionTimer.milliseconds() >= 250) {
                        robot.intake.setAction(Intake.IntakeActions.Wait);
                        robot.shooter.setTargetVelocity(3.5);
                        actionTimer.reset();
                        state = 8;
                    }
                    break;

                case 8:
                    if (robot.shooter.atTarget() && actionTimer.milliseconds() >= 250) {
                        robot.intake.setAction(Intake.IntakeActions.Collect);
                        actionTimer.reset();
                        state = 9;
                    } else if (actionTimer.seconds() > 1.5) {
                        robot.intake.setAction(Intake.IntakeActions.Collect);
                        actionTimer.reset();
                        state = 9;
                    }
                    break;

                case 9:
                    if (actionTimer.milliseconds() >= 300) {
                        robot.intake.setAction(Intake.IntakeActions.Collect);
                        robot.drive.followPath(goForSpike2, true);
                        robot.shooter.setTargetVelocity(-1);
                        pathTimer.reset();
                        state = 10;
                    }
                    break;

                case 10:
                    if ((robot.isDone()|| pathTimer.seconds() > 3.0) && actionTimer.milliseconds() >= 250) {
                        robot.drive.followPath(collectSpike2, true);
                        robot.drive.setMaxPower(0.5);
                        pathTimer.reset();
                        state = 11;
                    }
                    break;

                case 11:
                    if (robot.isDone()|| pathTimer.seconds() > 3.0) {
                        robot.drive.setMaxPower(1);
                        robot.drive.followPath(shootFromSpike2, true);
                        pathTimer.reset();
                        state = 12;
                    }
                    break;

                case 12:
                    if (robot.isDone()|| pathTimer.seconds() > 3.0) {
                        robot.intake.motorIntake.setPower(-0.7);
                        actionTimer.reset();
                        state = 13;
                    }
                    break;

                case 13:
                    if (actionTimer.milliseconds() >= 250) {
                        robot.intake.setAction(Intake.IntakeActions.Wait);
                        robot.shooter.setTargetVelocity(3.5);
                        actionTimer.reset();
                        state = 14;
                    }
                    break;

                case 14:
                    if (robot.shooter.atTarget() && actionTimer.milliseconds() >= 250) {
                        robot.intake.setAction(Intake.IntakeActions.Collect);
                        actionTimer.reset();
                        state = 15;
                    } else if (actionTimer.seconds() > 1.5) {
                        robot.intake.setAction(Intake.IntakeActions.Collect);
                        actionTimer.reset();
                        state = 15;
                    }
                    break;

                case 15:
                    if (robot.isDone()|| pathTimer.seconds() > 2.0) {
                        robot.intake.setAction(Intake.IntakeActions.Collect);
                        robot.drive.setMaxPower(0.5);
                        robot.drive.followPath(goForSpike3, true);
                        robot.shooter.setTargetVelocity(-1);
                        pathTimer.reset();
                        state = 16;
                    }
                    break;

                case 16:
                    if (robot.isDone()|| pathTimer.seconds() > 3.0) {
                        robot.drive.setMaxPower(1);
                        robot.drive.followPath(collectSpike3, true);
                        pathTimer.reset();
                        state = 17;
                    }
                    break;

                case 17:
                    if (robot.isDone()|| pathTimer.seconds() > 3.0) {
                        robot.drive.followPath(shootFromSpike3, true);
                        pathTimer.reset();
                        state = 18;
                    }
                    break;

                case 18:
                    if (robot.isDone()|| pathTimer.seconds() > 3.0) {
                        robot.intake.motorIntake.setPower(-0.7);
                        actionTimer.reset();
                        state = 19;
                    }
                    break;

                case 19:
                    if (actionTimer.milliseconds() >= 250) {
                        robot.intake.setAction(Intake.IntakeActions.Wait);
                        robot.shooter.setTargetVelocity(3.5);
                        actionTimer.reset();
                        state = 20;
                    }
                    break;

                case 20:
                    if (robot.shooter.atTarget() && actionTimer.milliseconds() >= 250) {
                        robot.intake.setAction(Intake.IntakeActions.Collect);
                        actionTimer.reset();
                        state = 21;
                    } else if (actionTimer.seconds() > 1.5) {
                        robot.intake.setAction(Intake.IntakeActions.Collect);
                        actionTimer.reset();
                        state = 21;
                    }
                    break;

                case 21:
                    if (robot.isDone()|| pathTimer.seconds() > 2.0) {
                        robot.drive.setMaxPower(1);
                        robot.drive.followPath(goToPark, true);
                        pathTimer.reset();
                        state = 22;
                    }
                    break;

                case 22:
                    if (robot.isDone()|| pathTimer.seconds() > 3.0) {
                        robot.drive.breakFollowing();
                        state = 99;
                    }
                    break;

                case 99:
                    break;
            }

            if(opModeTimer.seconds() > 28 && state != 21 && state != 22 && state != 99) {
                state = 21;
            }

            telemetry.addData("State", state);
            telemetry.addData("Path Timer", pathTimer.seconds());
            telemetry.addData("Action Timer", actionTimer.seconds());
            telemetry.addData("Robot Pose", robot.drive.getPose());
            telemetry.update();
        }
    }
}