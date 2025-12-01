package org.firstinspires.ftc.teamcode.Core.Module.Camera;

import static org.firstinspires.ftc.teamcode.Constants.CameraConstants.xOffset;
import static org.firstinspires.ftc.teamcode.Constants.CameraConstants.yOffset;
import static org.firstinspires.ftc.teamcode.Constants.DeviceNames.webcamName;
import static org.firstinspires.ftc.teamcode.Constants.Globals.randomizedCase;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Core.Hardware.HighModule;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;

@Config
public class Camera extends HighModule{

    public AprilTagProcessor aprilTagProcessor;
    public ArrayList<AprilTagDetection> detections = new ArrayList<>();
    ElapsedTime timer = new ElapsedTime();

    public Camera(HardwareMap hardwareMap){
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setLensIntrinsics(1385.92f,1385.92f,951.982f,534.084f)
                .setCameraPose(new Position(DistanceUnit.CM, 0, 0, 0,0), new YawPitchRollAngles(AngleUnit.DEGREES, 0,0,0,0))//TODO
                .setOutputUnits(DistanceUnit.CM, AngleUnit.RADIANS)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .build();
        VisionPortal camera = new VisionPortal.Builder()
                .setCameraResolution(new Size(1920,1080))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .setCamera(hardwareMap.get(WebcamName.class, webcamName))
                .addProcessor(aprilTagProcessor)
                .build();
        FtcDashboard.getInstance().startCameraStream(camera,60);
    }

    public int getAprilTagID(int index){
        if(!detections.isEmpty()){
            return detections.get(index).id;
        }
        return -1;
    }

    public Constants.Case getCase(){
        if(!detections.isEmpty()){
            for(int i = 0; i < detections.size(); i++) {
                switch (getAprilTagID(i)){
                    case 21:
                        randomizedCase = Constants.Case.GPP;
                        i=detections.size();
                        break;
                    case 22:
                        randomizedCase = Constants.Case.PGP;
                        i=detections.size();
                        break;
                    case 23:
                        randomizedCase = Constants.Case.PPG;
                        i=detections.size();
                        break;
                    default:
                        randomizedCase = Constants.Case.None;
                        break;
                }
            }
        }
        return randomizedCase;
    }

    public Pose distanceToAprilTag(){
        Pose pose = new Pose();
        int index = -1;
        AprilTagDetection detection = null;
        Vector vector = new Vector();
        if(!detections.isEmpty()){
            for(int i = 0; i < detections.size(); i++) {
                if (getAprilTagID(i) == 23 || getAprilTagID(i) ==24) {
                    index = i;
                    detection = detections.get(i);
                    break;
                }
            }
        }

        if(index != -1 && detection != null){
            vector = new Vector(new Pose(-detection.ftcPose.x,detection.ftcPose.y));
            vector.rotateVector(-detection.ftcPose.yaw);
            pose = new Pose(vector.getXComponent(), vector.getYComponent(), detection.ftcPose.yaw);
        }
        return pose;
    }

    public Pose targetPose(Pose currentPose){
        Pose aux = distanceToAprilTag();
        aux = new Pose(aux.getX()/2.54,aux.getY()/2.54,aux.getHeading());
        aux = aux.rotate(currentPose.getHeading() + aux.getHeading(), false);
        return new Pose(currentPose.getY() + aux.getY() + xOffset,currentPose.getX() + aux.getX() + yOffset, currentPose.getHeading() + aux.getHeading());
    }

    @Override
    public void update() {
        if(timer.milliseconds()>=100) {
            detections = aprilTagProcessor.getDetections();
            timer.reset();
        }
    }
}
