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

@Autonomous
public class AutoWithoutActions extends LinearOpMode {
    public Robot robot;
    private int state = 1, cycles = 0;

    public Pose startPose = new Pose(16, 112, Math.toRadians(0));
    private final Pose shootPose = new Pose(45, 102.5, Math.toRadians(-40));
    private final Pose spike1Pose = new Pose(55, 85, Math.toRadians(180));
    private final Pose collect1Pose = new Pose(22, 85, Math.toRadians(180));
    private final Pose spike2Pose = new Pose(55, 58, Math.toRadians(180));
    private final Pose collect2Pose = new Pose(16, 58, Math.toRadians(180));
    private final Pose spike3Pose = new Pose(55, 35, Math.toRadians(180));
    private final Pose collect3Pose = new Pose(16, 35, Math.toRadians(180));

    private final ElapsedTime timer = new ElapsedTime(), autoTimer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, startPose, true, Constants.Color.Blue, telemetry, gamepad1);

        final PathChain preloadPath = robot.drive.pathBuilder()
                .addPath(new BezierLine(startPose , shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(),shootPose.getHeading())
                .build();
        final PathChain goForSpike1 = robot.drive.pathBuilder()
                .addPath(new BezierLine(shootPose , spike1Pose))
                .setLinearHeadingInterpolation(shootPose.getHeading(),spike1Pose.getHeading())
                .build();
        final PathChain collectSpike1 = robot.drive.pathBuilder()
                .addPath(new BezierLine(spike1Pose , collect1Pose))
                .setLinearHeadingInterpolation(spike1Pose.getHeading(),collect1Pose.getHeading())
                .build();
        final PathChain shootFromSpike1 =robot.drive.pathBuilder()
                .addPath(new BezierLine(collect1Pose , shootPose))
                .setLinearHeadingInterpolation(collect1Pose.getHeading(),shootPose.getHeading())
                .build();
        final PathChain goForSpike2 =robot.drive.pathBuilder()
                .addPath(new BezierLine(shootPose , spike2Pose))
                .setLinearHeadingInterpolation(shootPose.getHeading(),spike2Pose.getHeading())
                .build();
        final PathChain collectSpike2 =robot.drive.pathBuilder()
                .addPath(new BezierLine(spike2Pose , collect2Pose))
                .setLinearHeadingInterpolation(spike2Pose.getHeading(),collect2Pose.getHeading())
                .build();
        final PathChain shootFromSpike2 = robot.drive.pathBuilder()
                .addPath(new BezierLine(collect2Pose , shootPose))
                .setLinearHeadingInterpolation(collect2Pose.getHeading(),shootPose.getHeading())
                .build();
        final PathChain goForSpike3 =robot.drive.pathBuilder()
                .addPath(new BezierLine(shootPose , spike3Pose))
                .setLinearHeadingInterpolation(shootPose.getHeading(),spike3Pose.getHeading())
                .build();
        final PathChain collectSpike3 =robot.drive.pathBuilder()
                .addPath(new BezierLine(spike3Pose , collect3Pose))
                .setLinearHeadingInterpolation(spike3Pose.getHeading(),collect3Pose.getHeading())
                .build();
        final PathChain shootFromSpike3 =robot.drive.pathBuilder()
                .addPath(new BezierLine(collect3Pose , shootPose))
                .setLinearHeadingInterpolation(collect3Pose.getHeading(),shootPose.getHeading())
                .build();
        final PathChain goToPark =robot.drive.pathBuilder()
                .addPath(new BezierLine(shootPose , collect1Pose))
                .setLinearHeadingInterpolation(shootPose.getHeading(),collect1Pose.getHeading())
                .build();


        telemetry.addLine("Done");
        telemetry.update();
        waitForStart();
        autoTimer.reset();
        while (opModeIsActive()) {
            switch (state) {
                case 1:
                    robot.drive.followPath(preloadPath, true);
                    robot.shooter.setTargetVelocity(3.5);
                    timer.reset();
                    state = 3;
                    break;
                case 3:
                    if(robot.isDone()){
                        robot.intake.setAction(Intake.IntakeActions.Wait);
                        timer.reset();
                        state = 5;
                        cycles = 0;
                    }
                    break;
                case 5:
                    if(robot.shooter.atTarget() && timer.milliseconds() >= 250){
                        robot.intake.setAction(Intake.IntakeActions.Collect);
                        timer.reset();
                        state++;
                        cycles++;
                        if(cycles == 4){
                            state = 7;
                        }
                    }
                    break;
                case 6: //TODO FailSafe
                    if(timer.milliseconds() >= 1200 || robot.shooter.getVelocityError() >= 0.7){
                        robot.intake.setAction(Intake.IntakeActions.Wait);
                        timer.reset();
                        state = 5;
                    }
                    break;
                case 7:
                    if(timer.milliseconds()>= 300){
                        robot.intake.setAction(Intake.IntakeActions.Collect);
                        robot.drive.setMaxPower(1);
                        robot.drive.followPath(goForSpike1, true);
                        robot.shooter.setTargetVelocity(-1);
                        timer.reset();
                        state++;
                    }
                    break;
                case 8:
                    if(robot.isDone() && timer.milliseconds() >= 250){
                        robot.drive.followPath(collectSpike1, true);
                        robot.drive.setMaxPower(0.5);
                        state++;
                    }
                    break;
                case 9:
                    if(robot.isDone()){
                        robot.drive.setMaxPower(1);
                        robot.drive.followPath(shootFromSpike1, true);
                        state++;
                    }
                    break;
                case 10:
                    if(robot.isDone()){
                        robot.intake.motorIntake.setPower(-0.7);
                        timer.reset();
                        state++;
                    }
                    break;
                case 11:
                    if(timer.milliseconds() >= 250){
                        robot.intake.setAction(Intake.IntakeActions.Wait);
                        robot.shooter.setTargetVelocity(3.5);
                        timer.reset();
                        state++;
                        cycles = 0;
                    }
                    break;
                case 12:
                    if(robot.shooter.atTarget() && timer.milliseconds() >= 250){
                        robot.intake.setAction(Intake.IntakeActions.Collect);
                        timer.reset();
                        state++;
                        cycles++;
                        if(cycles == 4){
                            state = 14;
                        }
                    }
                    break;
                case 13: //TODO FailSafe
                    if(timer.milliseconds() >= 1200 || robot.shooter.getVelocityError() >= 0.7){
                        robot.intake.setAction(Intake.IntakeActions.Wait);
                        timer.reset();
                        state = 12;
                    }
                    break;
                case 14:
                    if(robot.isDone()){
                        robot.intake.setAction(Intake.IntakeActions.Collect);
                        robot.drive.followPath(goForSpike2, true);
                        robot.shooter.setTargetVelocity(-1);
                        timer.reset();
                        state++;
                    }
                    break;
//                case 15:
//                    if(robot.isDone() && timer.milliseconds() >= 250){
//                        robot.drive.followPath(collectSpike2, true);
//                        robot.drive.setMaxPower(0.5);
//                        state++;
//                    }
//                    break;
//                case 16:
//                    if(robot.isDone()){
//                        robot.drive.setMaxPower(1);
//                        robot.drive.followPath(shootFromSpike2, true);
//                        state++;
//                    }
//                    break;
//                case 17:
//                    if(robot.isDone()){
//                        robot.intake.motorIntake.setPower(-0.7);
//                        timer.reset();
//                        state++;
//                    }
//                    break;
//                case 18:
//                    if(timer.milliseconds() >= 250){
//                        robot.intake.setAction(Intake.IntakeActions.Wait);
//                        robot.shooter.setTargetVelocity(3.5);
//                        timer.reset();
//                        state++;
//                        cycles = 0;
//                    }
//                    break;
//                case 19:
//                    if(robot.shooter.atTarget() && timer.milliseconds() >= 250){
//                        robot.intake.setAction(Intake.IntakeActions.Collect);
//                        timer.reset();
//                        state++;
//                        cycles++;
//                        if(cycles == 4){
//                            state = 21;
//                        }
//                    }
//                    break;
//                case 20: //TODO FailSafe
//                    if(timer.milliseconds() >= 1200 || robot.shooter.motorUp.getCurrentVelocity() <= 3){
//                        robot.intake.setAction(Intake.IntakeActions.Wait);
//                        timer.reset();
//                        state = 19;
//                    }
//                    break;
//                case 21:
//                    if(robot.isDone()){
//                        robot.intake.setAction(Intake.IntakeActions.Collect);
//                        robot.drive.setMaxPower(0.5);
//                        robot.drive.followPath(goForSpike3, true);
//                        robot.shooter.setTargetVelocity(-1);
//                        state++;
//                    }
//                    break;
//                case 22:
//                    if(robot.isDone()){
//                        robot.drive.setMaxPower(1);
//                        robot.drive.followPath(collectSpike3, true);
//                        state++;
//                    }
//                    break;
//                case 23:
//                    if(robot.isDone()){
//                        robot.drive.followPath(shootFromSpike3, true);
//                        state++;
//                    }
//                    break;
//                case 24:
//                    if(robot.isDone()){
//                        robot.intake.motorIntake.setPower(-0.7);
//                        timer.reset();
//                        state++;
//                    }
//                    break;
//                case 25:
//                    if(timer.milliseconds() >= 250){
//                        robot.intake.setAction(Intake.IntakeActions.Wait);
//                        robot.shooter.setTargetVelocity(3.5);
//                        timer.reset();
//                        state++;
//                        cycles = 0;
//                    }
//                    break;
//                case 26:
//                    if(robot.shooter.atTarget() && timer.milliseconds() >= 250){
//                        robot.intake.setAction(Intake.IntakeActions.Collect);
//                        timer.reset();
//                        state++;
//                        cycles++;
//                        if(cycles == 4){
//                            state = 28;
//                        }
//                    }
//                    break;
//                case 27: //TODO FailSafe
//                    if(timer.milliseconds() >= 1200 || robot.shooter.getVelocityError() >= 0.7){
//                        robot.intake.setAction(Intake.IntakeActions.Wait);
//                        timer.reset();
//                        state = 26;
//                    }
//                    break;
//                case 28:
//                    if(robot.isDone()){
//                        robot.drive.setMaxPower(1);
//                        robot.drive.followPath(goToPark, true);
//                        state++;
//                    }
//                    break;

            }
            telemetry.addData("state", state);
            telemetry.addData("robot pose", robot.drive.getPose());
            telemetry.addData("shooter velocity", robot.shooter.motorUp.getCurrentVelocity());
            telemetry.addData("shooter target", robot.shooter.motorUp.getTarget());
            telemetry.addData("shooter power", robot.shooter.motorUp.getPower());
            telemetry.addData("intake current", robot.intake.motorIntake.motor.getCurrentDrawn());
            telemetry.update();

            if(autoTimer.milliseconds() >= 27500 && state != 28 && state != 15){
                state = 28;
            }
            robot.update();
        }


    }

}
