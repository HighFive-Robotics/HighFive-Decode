package org.firstinspires.ftc.teamcode.Core.Hardware;

import static org.firstinspires.ftc.teamcode.Constants.DeviceNames.cameraName;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

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
    public HighCamera(@NonNull HardwareMap hardwareMap , @NonNull Pipelines pipeline){
        ll = hardwareMap.get(Limelight3A.class , cameraName);
        ll.setPollRateHz(100);
        ll.pipelineSwitch(pipeline.getPipelineNumber());
    }
    public void startCapture(){
        ll.start();
    }
    public void stopCapture(){
        ll.stop();
    }
    public void setPipeline(@NonNull Pipelines pipeline){
        ll.pipelineSwitch(pipeline.getPipelineNumber());
    }
    public LLResult getResult(){
        LLResult result = ll.getLatestResult();
        if(result != null && result.isValid()){
            return result;
        }else return null;
    }
    public int getAprilTagId(){
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
    public boolean motifIsValid(Constants.Case motif){
        return motif != Constants.Case.None;
    }
    public boolean resultIsValid(LLResult result){
        return result != null;
    }
}
