
package org.firstinspires.ftc.teamcode.Core.Module.Camera;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Core.Robot;

import java.util.List;

@TeleOp(name = "Sensor: Limelight3A", group = "Sensor")
@Disabled
public class LimeLightPoseConverter extends LinearOpMode {

    private Limelight3A limelight;

    @Override
    public void runOpMode() throws InterruptedException
    {
        Robot robot = new Robot(hardwareMap,new Pose(0,0,0) , false , Constants.Color.Red , telemetry,gamepad1);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0);
        limelight.start();

        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            LLStatus status = limelight.getStatus();
            telemetry.addData("Name", "%s",
                    status.getName());
            telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                    status.getTemp(), status.getCpu(),(int)status.getFps());
            telemetry.addData("Pipeline", "Index: %d, Type: %s",
                    status.getPipelineIndex(), status.getPipelineType());
            limelight.updateRobotOrientation(robot.drive.getHeading());
            LLResult result = limelight.getLatestResult();
            if (result.isValid()) {
                Pose3D botpose = result.getBotpose();
                double captureLatency = result.getCaptureLatency();
                double targetingLatency = result.getTargetingLatency();
                telemetry.addData("LL Latency", captureLatency + targetingLatency);
                double rotationError = result.getTx();
                telemetry.addData("Rotation Error (tx)", "%.2f degrees", rotationError);
                Pose3D targetPose = result.getBotpose_MT2();
                double forwardDistance = targetPose.getPosition().x;
                double strafeDistance = targetPose.getPosition().y;
                double rotation = targetPose.getOrientation().getYaw(AngleUnit.RADIANS);
                telemetry.addData("Forward Distance (X)", "%.2f units", forwardDistance);
                telemetry.addData("Strafe Distance (Y)", "%.2f units (positive is left)", strafeDistance);
                telemetry.addData("Botpose (Field-Relative)", botpose.toString());
                telemetry.addData("tx", result.getTx());
                telemetry.addData("txnc", result.getTxNC());
                telemetry.addData("ty", result.getTy());
                telemetry.addData("tync", result.getTyNC());
                telemetry.addData("Botpose", botpose.toString());
                // Access fiducial results
                List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                for (LLResultTypes.FiducialResult fr : fiducialResults) {
                    telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
                }
            } else {
                telemetry.addData("Limelight", "No data available");
            }
            telemetry.update();
        }
        limelight.stop();
    }
}