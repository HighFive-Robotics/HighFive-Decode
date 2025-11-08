package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Constants.DeviceNames.pinPointName;
import static org.firstinspires.ftc.teamcode.Constants.DeviceNames.rightBackMotorName;
import static org.firstinspires.ftc.teamcode.Constants.DeviceNames.rightFrontMotorName;

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
        public static String shooterMotorUpName = "OMT";
        public static String shooterMotorDownName = "OMB";
    }

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static FollowerConstants FConstants = new FollowerConstants()
            .mass(8)
            .forwardZeroPowerAcceleration(-38.72073)
            .lateralZeroPowerAcceleration(-67.852369)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.05,0,0.0052,0.025))
            .headingPIDFCoefficients(new PIDFCoefficients(1,0,0.09,0.01))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.015,0,0.00015,0.6,0.01))
            .centripetalScaling(0.0008)
            ;

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName(rightFrontMotorName)
            .rightRearMotorName(rightBackMotorName)
            .leftRearMotorName(DeviceNames.leftBackMotorName)
            .leftFrontMotorName(DeviceNames.leftFrontMotorName)
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .xVelocity(90.6866)
            .yVelocity(71.773132)
            ;

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(0)
            .strafePodX(7.25)
            .distanceUnit(DistanceUnit.CM)
            .hardwareMapName(pinPointName)
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(FConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}
