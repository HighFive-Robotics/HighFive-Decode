package org.firstinspires.ftc.teamcode.Core.Hardware;

import static org.firstinspires.ftc.teamcode.Constants.Color.Green;
import static org.firstinspires.ftc.teamcode.Constants.Color.None;
import static org.firstinspires.ftc.teamcode.Constants.Color.Purple;
import static org.firstinspires.ftc.teamcode.Constants.Intake.ColorSensorConstants.GreenValuesHSV;
import static org.firstinspires.ftc.teamcode.Constants.Intake.ColorSensorConstants.PurpleValuesHSV;
import static org.firstinspires.ftc.teamcode.Constants.Intake.ColorSensorConstants.currentColor;
import static org.firstinspires.ftc.teamcode.Constants.Intake.ColorSensorConstants.targetGreenRGB;
import static org.firstinspires.ftc.teamcode.Constants.Intake.ColorSensorConstants.targetPurpleRGB;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Core.Algorithms.LowPassFilter;

import java.util.Arrays;

@Config
public class HighSensor extends HighModule {
    RevColorSensorV3 sensor;
    private final LowPassFilter hFilter, sFilter, vFilter;

    public static double SAT = 0.15;
    public static double minSaturation = 0.2;

    float[] hsvValues = new float[3];

    public HighSensor(HardwareMap hardwareMap, String name) {
        sensor = hardwareMap.get(RevColorSensorV3.class, name);
        sensor.setGain(25.0f);

        hFilter = new LowPassFilter(0.7, 0);
        sFilter = new LowPassFilter(0.7, 0);
        vFilter = new LowPassFilter(0.7, 0);

        float[] tempHsv = new float[3];
    }

    public double getDistance(DistanceUnit distanceUnit) {
        return sensor.getDistance(distanceUnit);
    }

    private void updateColorProcessing() {
        NormalizedRGBA colors = sensor.getNormalizedColors();
        Color.colorToHSV(colors.toColor(), hsvValues);

        hsvValues[0] = (float) hFilter.getValue(hsvValues[0]);
        hsvValues[1] = (float) sFilter.getValue(hsvValues[1]);
        hsvValues[2] = (float) vFilter.getValue(hsvValues[2]);

    }

    private void determineColorState() {
        if (hsvValues[1] < minSaturation) {
            currentColor = None;
            return;
        }

        if (Math.abs(hsvValues[1] - GreenValuesHSV[1]) <= SAT) {
            currentColor = Green;
        }
        else if (Math.abs(hsvValues[1] - PurpleValuesHSV[0]) <= SAT) {
            currentColor = Purple;
        }
        else {
            currentColor = None;
        }
    }

    @Override
    public void update() {
        updateColorProcessing();
        determineColorState();
    }

    public Constants.Color getColor() {
        return currentColor;
    }

    public float[] getHSVColorValues(){
        return hsvValues;
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Distance (CM)", sensor.getDistance(DistanceUnit.CM));
        telemetry.addData("HSV", Arrays.toString(hsvValues));
        telemetry.addData("Hue", hsvValues[0]);
        telemetry.addData("Saturation", hsvValues[1]);
        telemetry.addData("Value", hsvValues[2]);
        telemetry.addData("Green Target Hue", GreenValuesHSV[1]);
        telemetry.addData("Purple Target Hue", PurpleValuesHSV[1]);
        telemetry.addData("Detected Color", currentColor);
    }
}