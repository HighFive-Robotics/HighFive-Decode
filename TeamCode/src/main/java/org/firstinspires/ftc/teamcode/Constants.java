package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Constants.DeviceNames.pinPointName;
import static org.firstinspires.ftc.teamcode.Constants.DeviceNames.rightBackMotorName;
import static org.firstinspires.ftc.teamcode.Constants.DeviceNames.rightFrontMotorName;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
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

public class Constants {

    @Config
    public static class Globals{
        public static double voltage = 12.0;
        public static Pose finalAutoPose = new Pose(0,0,0);
        public static Color autoColor = Color.Red;
        public static Case randomizedCase = Case.None;
        public static Pose BlueGoal = new Pose(16,118), RedGoal = new Pose(120,124);
    }

    public enum Case {
        GPP,
        PGP,
        PPG,
        None
    }

    public enum Color {
        Blue,
        //Yellow, gg bye bye Itd
        Red,
        Green,
        Purple,
        None
    }

    public static class DeviceNames {
        public static String leftFrontMotorName = "LFM";
        public static String leftBackMotorName = "LBM";
        public static String rightFrontMotorName = "RFM";
        public static String rightBackMotorName = "RBM";
        public static String pinPointName = "odo";
        public static String webcamName = "Webcam 1";
        public static String intakeMotorName = "IM";
        public static String intakeSensorName = "IS";
        public static String breakBeamIntakeName = "IB";
        public static String shooterMotorUpName = "OMT";
        public static String shooterMotorDownName = "OMB";
        public static String blockerServoName = "BO";
        public static String sorterServoName = "S";
        public static String sorterAnalogInputName = "SAI";
        public static String liftServoLB = "SLB";
        public static String liftServoRB = "SRB";
        public static String liftServoRF = "SRF";
        public static String ledName = "L";
        public static String cameraName = "limelight";
    }

    public static class Intake{
        @Config
        public static class IntakePowers{
            public static double powerWait = 0, powerCollect = 1, powerSpit = -1, powerTransfer = 1;
        }

        @Config
        public static class ColorSensorConstants{
            public static Color currentColor = Color.None;

            public static float[] GreenValuesHSV = {125.0F,0.6F,20F};
            public static float[] PurpleValuesHSV = {200F,0.4F,5F};
        }

        @Config
        public static class SorterConstants{

            public static double kP = 0.00055,kD=0.00006,kI=0,
                    staticGain=0.068,dynamicGain=0,inertialGain=0,
                    bangBangZone=100, precisionZone= 10 ,slewRate=5,
                    tolerance =2 , lookAhead= 0.05 , breakingGain = 2.0,
                    spinOpposingGain=1.2 , spinOpposingPrefix=-1.0,
                    bangBangGain = 0.95 , additionMin = 0.5 , additionMax=1,
                    viscousGain = 2.7;
            public static boolean shouldUpdateCoef = false;
            public static int artifactNumber = 0, purpleArtifactNumber = 0, greenArtifactNumber = 0;
            public static double ticksPerRotation = 8192;
            public static double targetSlot1 = 0, targetSlot2 = 120, targetSlot3 = 240;
            public static Color[][] targetColors = {{Color.Green, Color.Purple,Color.Purple},{Color.Purple,Color.Green,Color.Purple},{ Color.Purple,Color.Purple,Color.Green}};
            public static Color[] sorterColors = {Color.None,Color.None,Color.None};
        }
    }

    @Config
    public static class CameraConstants{
        public static double xOffset = 0, yOffset = 0;
    }

    @Config
    public static class ShooterConstants{
        public static double kp = 0.0007, kd = 5e-8,ki = 0.005,kf = 0.00022,ks=0,ka=0;
        public static final double wheelDiameter = 0.072;
        public static final double encoderResolution = 28;

    }

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static FollowerConstants FConstants = new FollowerConstants()
            .mass(15.5)
            .forwardZeroPowerAcceleration(-36.6917641)
            .lateralZeroPowerAcceleration(-62.2850712)
            .useSecondaryTranslationalPIDF(false)
            .useSecondaryHeadingPIDF(false)
            .useSecondaryDrivePIDF(true)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.125,0,0.0175,0))
            .headingPIDFCoefficients(new PIDFCoefficients(2,0,0.15,0))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(3,0,0.05,0))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.1,0, 0.001,0.6,0.01))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.01,0, 0.0001,0.6,0.01))
            .centripetalScaling(0.0004)
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
            .xVelocity(80.185892)
            .yVelocity(63.103582)
            ;

    public static PinpointConstants localizerConstants = new PinpointConstants()
//            .distanceUnit(DistanceUnit.CM)
            .forwardPodY(-2.8)
            .strafePodX(-6.48)
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
