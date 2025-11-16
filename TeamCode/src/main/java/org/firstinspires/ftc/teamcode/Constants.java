package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Constants.DeviceNames.pinPointName;
import static org.firstinspires.ftc.teamcode.Constants.DeviceNames.rightBackMotorName;
import static org.firstinspires.ftc.teamcode.Constants.DeviceNames.rightFrontMotorName;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {

    public static class Globals{
        public static double voltage = 12.0;
    }

    public enum Case {
        Left,
        Middle,
        Right,
        None
    }

    public static Case randomizedCase = Case.None;

    public enum Color {
        Blue,
        //Yellow, gg bye bye Itd
        Red,
        Green,
        Purple,
        None
    }

    public static Color[][] targetColors = {{Color.Green, Color.Purple,Color.Purple},{Color.Purple,Color.Green,Color.Purple},{ Color.Purple,Color.Purple,Color.Green}};
    public static Color[] mixerColors = {Color.None,Color.None,Color.None};
    public static Color currentColor = Color.None;
    public static double GreenValuesHSV[] = {160F,0.75F,20F};
    public static double PurpleValuesHSV[] = {215F,0.40F,5F};
    public static double Treshold[] = {17.5F, 0.2F, 5F};


    public static class DeviceNames {
        public static String leftFrontMotorName = "LFM";
        public static String leftBackMotorName = "LBM";
        public static String rightFrontMotorName = "RFM";
        public static String rightBackMotorName = "RBM";
        public static String pinPointName = "odo";
        public static String webcamName = "Webcam 1";
        public static String intakeMotorName = "IM";
        public static String intakeJointServoName = "IJ";
        public static String shooterMotorUpName = "OMT";
        public static String shooterMotorDownName = "OMB";
        public static String blockerServoName = "BO";
        public static String sorterServoName = "S";
        public static String liftServoLB = "SLB";
        public static String liftServoRB = "SRB";
        public static String liftServoRF = "SRF";
    }

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static FollowerConstants FConstants = new FollowerConstants()
            .mass(16.5)
            .forwardZeroPowerAcceleration(-42)
            .lateralZeroPowerAcceleration(-55)

            ;

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName(rightFrontMotorName)
            .rightRearMotorName(rightBackMotorName)
            .leftRearMotorName(DeviceNames.leftBackMotorName)
            .leftFrontMotorName(DeviceNames.leftFrontMotorName)
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(76.775)
            .yVelocity(58.2)
            ;

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(7)
            .strafePodX(16.45)
            .hardwareMapName(pinPointName)
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(FConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}
