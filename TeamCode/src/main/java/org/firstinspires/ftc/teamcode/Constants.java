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
    public static class Globals {
        public static double voltage = 12.0;
        public static boolean afterAuto = false;
        public static Pose finalAutoPose = new Pose(0, 0, 0);
        public static Color autoColor = Color.Red;
        public static Case randomizedCase = Case.None;
        public static Pose BlueGoalCorner = new Pose(4.5, 139.5), BlueGoalWallUp = new Pose(14, 132), BlueGoalWallLeft = new Pose(12, 130), BlueGoalDistance = new Pose(18.5, 123.5);
        public static Pose RedGoalCorner = new Pose(139.5, 139.5), RedGoalWallUp = new Pose(130, 132), RedGoalWallRight = new Pose(132, 130), RedGoalDistance = new Pose(121, 121);
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
        public static String breakBeamIntakeNameUp = "IBU";
        public static String breakBeamIntakeNameDown = "IBD";
        public static String breakBeamOuttakeName = "OB";
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
            public static double kpFly = 0.004, kdFly = 0.0001, kiFly = 0.0035, kfFly = 0.00016, ksFly = 0, kaFly = 0;
            public static final double wheelDiameterFly = 0.096;
            public static final double encoderResolutionFly = 28;
        }
        @Config
        public static class ShooterBackWheelParams {
            public static double kpBack = 0.00093, kdBack = 0.00002, kiBack = 0.0009, kfBack = 0.00008, ksBack = 0, kaBack = 0;
            public static final double wheelDiameterBack = 0.048;
            public static final double encoderResolutionBack = 28;
            public static double kC = 1;
        }
        @Config
        public static class TurretParams {
            public static double kpTurret = 0.026, kdTurret = 0.0012, kiTurret = 0.026, kfTurret = 0.0001;
            public static double kp2Turret = 0.026, kd2Turret = 0.0012, ki2Turret = 0.026, thresholdSmallPID = 35;
            public static double minimumTicks = -502, maximumTicks = 502;
            public static double ticksPerPI = 430.0;
            public static double minimumErrorAngleForWalls = Math.PI / 10;
        }

    }
    @Config
    public static class CameraConstants {
        public static double xOffset = 0, yOffset = 0;
    }


    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static FollowerConstants FConstants = new FollowerConstants()
            .holdPointHeadingScaling(1)
            .mass(13.7)
            .forwardZeroPowerAcceleration(-30.7325319)
            .lateralZeroPowerAcceleration(-74.2480333)
            .useSecondaryTranslationalPIDF(false)
            .useSecondaryHeadingPIDF(false)
            .useSecondaryDrivePIDF(true)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.125, 0, 0.0175, 0))
            .headingPIDFCoefficients(new PIDFCoefficients(2, 0, 0.15, 0))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(2.5, 0, 0.07, 0))
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
            .xVelocity(78.989683)
            .yVelocity(61.149199);

    public static PinpointConstants localizerConstants = new PinpointConstants()
           //.distanceUnit(DistanceUnit.CM)
            .forwardPodY(2.16)//-55mm 2,16inch
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

