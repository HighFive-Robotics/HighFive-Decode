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

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {

    @Config
    public static class Globals {
        public static double voltage = 12.0;
        public static boolean afterAuto = false;
        public static Pose finalAutoPose = new Pose(0, 0, 0);
        public static Color autoColor = Color.Red;
        public static Case randomizedCase = Case.None;
        public static Pose BlueGoalCorner = new Pose(10, 134), BlueGoalWallUp = new Pose(12, 134), BlueGoalWallLeft = new Pose(10, 132), BlueGoalDistance = new Pose(22.5, 123);
        public static Pose RedGoalCorner = new Pose(134, 134), RedGoalWallUp = new Pose(132, 134), RedGoalWallRight = new Pose(134, 132), RedGoalDistance = new Pose(126, 122);
    }

    public enum Case {
        GPP,
        PGP,
        PPG,
        None
    }

    public enum Color {
        Blue,
        Yellow,
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
        public static String breakBeamIntakeNameUp = "IB";
        public static String breakBeamIntakeNameDown = "IB";
        public static String shooterMotorUpName = "OMT";
        public static String shooterMotorDownName = "OMB";
        public static String turretMotorName = "TM";
        public static String blockerServoName = "BO";
        public static String ledName1 = "L1";
        public static String ledName2 = "L2";
        public static String cameraName = "limelight";
    }

    public static class Intake {
        @Config
        public static class IntakePowers {
            public static double powerWait = 0, powerCollect = 1, powerSpit = -1, powerTransfer = 1;
        }

        @Config
        public static class ColorSensorConstants {
            public static Color currentColor = Color.None;

            public static float[] GreenValuesHSV = {125.0F, 0.6F, 20F};
            public static float[] PurpleValuesHSV = {200F, 0.4F, 5F};
        }
    }

    public static class OuttakeConstants {

        @Config
        public static class ShooterFlyWheelParams {
            public static double kpFly = 0.00085, kdFly = 0.00006, kiFly = 0.0038, kfFly = 0.00016, ksFly = 0, kaFly = 0;
            public static final double wheelDiameterFly = 0.096;
            public static final double encoderResolutionFly = 28;
        }
        @Config
        public static class ShooterBackWheelParams {
            public static double kpBack = 0.0006, kdBack = 0.00002, kiBack = 0.0007, kfBack = 0.00016, ksBack = 0, kaBack = 0;
            public static final double wheelDiameterBack = 0.048;
            public static final double encoderResolutionBack = 145.1;
        }
        @Config
        public static class TurretParams {
            public static double kpTurret = 0.0075, kdTurret = 0.0002, kiTurret = 0.005, kfTurret = 0.0001;
            public static double minimumTicks = -1375, maximumTicks = 1375, ticksPerPI = 1125;
        }

    }
    @Config
    public static class CameraConstants {
        public static double xOffset = 0, yOffset = 0;
    }


    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static FollowerConstants FConstants = new FollowerConstants()
            .holdPointHeadingScaling(1)
            .mass(15.5)
            .forwardZeroPowerAcceleration(-36.6917641)
            .lateralZeroPowerAcceleration(-62.2850712)
            .useSecondaryTranslationalPIDF(false)
            .useSecondaryHeadingPIDF(false)
            .useSecondaryDrivePIDF(true)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.125, 0, 0.0175, 0))
            .headingPIDFCoefficients(new PIDFCoefficients(2, 0, 0.15, 0))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(3, 0, 0.05, 0))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.1, 0, 0.001, 0.6, 0.01))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.01, 0, 0.0001, 0.6, 0.01))
            .centripetalScaling(0.0004);

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
            .yVelocity(63.103582);

    public static PinpointConstants localizerConstants = new PinpointConstants()
           //.distanceUnit(DistanceUnit.CM)
            .forwardPodY(-2.16)//-55mm -2,16inch
            .strafePodX(-3.35)//-85mm -3,35 inch
            .hardwareMapName(pinPointName)
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(FConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}

