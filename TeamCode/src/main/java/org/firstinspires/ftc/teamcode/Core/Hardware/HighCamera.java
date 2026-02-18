package org.firstinspires.ftc.teamcode.Core.Hardware;

import static org.firstinspires.ftc.teamcode.Constants.DeviceNames.cameraName;

import androidx.annotation.NonNull;

import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Constants;

import java.util.List;

public class HighCamera{
    public enum Pipelines{
        AprilTagId(0),
        AprilTagLocation(1),
        BallDetection(8);
        final int pipelineNumber;
        Pipelines(int pipelineNumber){
            this.pipelineNumber = pipelineNumber;
        }
        int getPipelineNumber(){return pipelineNumber;}
    };
    Limelight3A ll;
    Pipelines pipeline;
    public HighCamera(@NonNull HardwareMap hardwareMap , @NonNull Pipelines pipeline){
        ll = hardwareMap.get(Limelight3A.class , cameraName);
        ll.setPollRateHz(100);
        setPipeline(pipeline);
    }
    public void startCapture(){
        ll.start();
    }
    public void stopCapture(){
        ll.stop();
    }
    public void setPipeline(@NonNull Pipelines pipeline){
        ll.pipelineSwitch(pipeline.getPipelineNumber());
        this.pipeline = pipeline;
    }
    public LLResult getResult(){
        LLResult result = ll.getLatestResult();
        if(result != null && result.isValid()){
            return result;
        }else return null;
    }
    public int getAprilTagId(){
        if(pipeline != Pipelines.AprilTagId) setPipeline(Pipelines.AprilTagId);
        LLResult result = getResult();
        if(resultIsValid(result)){
            List<LLResultTypes.FiducialResult> aprilTags = result.getFiducialResults();
            return aprilTags.get(0).getFiducialId();
        }else return -1;
    }
    public Constants.Case getMotif(){
        int id = getAprilTagId();
        switch (id){
            case 21:
                return Constants.Case.GPP;
            case 22:
                return Constants.Case.PGP;
            case 23:
                return Constants.Case.PPG;
            default:
                return Constants.Case.None;
        }
    }
    public Pose getAprilTagPose(){
        if (pipeline != Pipelines.AprilTagLocation) setPipeline(Pipelines.AprilTagLocation);
        LLResult result = getResult();
        if(resultIsValid(result)){
            LLResultTypes.FiducialResult fr = result.getFiducialResults().get(0);
            double poseX = fr.getRobotPoseTargetSpace().getPosition().z * 39.37;
            double poseY = -fr.getRobotPoseTargetSpace().getPosition().x * 39.37;
            double poseHeading = Math.toRadians(-fr.getRobotPoseTargetSpace().getOrientation().getYaw());
            return new Pose(poseX,poseY,poseHeading);
        }
        return null;
    }
    public Pose getAprilTagPose(double heading){
        if (pipeline != Pipelines.AprilTagLocation) setPipeline(Pipelines.AprilTagLocation);
        ll.updateRobotOrientation(heading);
        LLResult result = getResult();
        if(resultIsValid(result)){
            LLResultTypes.FiducialResult fr = result.getFiducialResults().get(0);
            double poseX = -fr.getRobotPoseTargetSpace().getPosition().x * 39.37;
            double poseY = fr.getRobotPoseTargetSpace().getPosition().z * 39.37;
            double poseHeading = Math.toRadians(-fr.getRobotPoseTargetSpace().getOrientation().getYaw());
            return new Pose(poseX,poseY,poseHeading);
        }
        return null;
    }
    public Pose getMegaTagFieldPose() {
        LLResult result = getResult();
        if (result != null) {
            Pose3D mt1 = result.getBotpose();
            if (mt1 != null) {
                // Convert meters to inches and shift the center (0,0) to the corner (0,0) by adding 72
                double xInches = (mt1.getPosition().x * 39.3701) + 72.0;
                double yInches = (mt1.getPosition().y * 39.3701) + 72.0;
                double headingRadians = Math.toRadians(mt1.getOrientation().getYaw());

                return new Pose(xInches, yInches, headingRadians);
            }
        }
        return null;
    }

    public Pose getMegaTagFieldPose(double heading) {
        ll.updateRobotOrientation(heading);
        LLResult result = getResult();
        if (result != null) {
            Pose3D mt1 = result.getBotpose();
            if (mt1 != null) {
                // Convert meters to inches and shift the center (0,0) to the corner (0,0) by adding 72
                double xInches = (mt1.getPosition().x * 39.3701) + 72.0;
                double yInches = (mt1.getPosition().y * 39.3701) + 72.0;
                double headingRadians = Math.toRadians(mt1.getOrientation().getYaw());

                return new Pose(xInches, yInches, headingRadians);
            }
        }
        return null;
    }

    public Pose getBallPose() {
        if (pipeline != Pipelines.BallDetection) setPipeline(Pipelines.BallDetection);
        LLResult result = getResult();
        if (resultIsValid(result)) {
            double[] py = result.getPythonOutput();

            if (py[0] == 1.0) {
                double forwardInch = py[1];
                double strafeInch = py[2];
                double headingDeg = py[3];
                return new Pose(forwardInch, strafeInch, Math.toRadians(headingDeg));
            }
        }
        return null;
    }
    public boolean motifIsValid(Constants.Case motif){
        return motif != Constants.Case.None;
    }
    public boolean resultIsValid(LLResult result){
        return result != null && result.isValid();
    }
}
