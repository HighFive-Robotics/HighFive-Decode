package org.firstinspires.ftc.teamcode.Core.Hardware;

import static org.firstinspires.ftc.teamcode.Constants.Color.Green;
import static org.firstinspires.ftc.teamcode.Constants.Color.None;
import static org.firstinspires.ftc.teamcode.Constants.Color.Purple;
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

@Config
public class HighSensor extends HighModule {
    RevColorSensorV3 sensor;
    private final LowPassFilter hFilter, sFilter, vFilter;

    public static double hueTolerance = 30;
    public static double minSaturation = 0.3;
    public static double maxDistanceCm = 3.0;

    float[] hsvValues = new float[3];
    double filteredHue = 0;

    private double greenTargetHue;
    private double purpleTargetHue;

    public HighSensor(HardwareMap hardwareMap, String name) {
        sensor = hardwareMap.get(RevColorSensorV3.class, name);
        sensor.setGain(25.0f);

        hFilter = new LowPassFilter(0.7, 0);
        sFilter = new LowPassFilter(0.7, 0);
        vFilter = new LowPassFilter(0.7, 0);

        float[] tempHsv = new float[3];
        Color.RGBToHSV((int)targetGreenRGB[0], (int)targetGreenRGB[1], (int)targetGreenRGB[2], tempHsv);
        greenTargetHue = tempHsv[0];

        Color.RGBToHSV((int)targetPurpleRGB[0], (int)targetPurpleRGB[1], (int)targetPurpleRGB[2], tempHsv);
        purpleTargetHue = tempHsv[0];
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

        filteredHue = hsvValues[0];
    }

    private void determineColorState() {
        double dist = sensor.getDistance(DistanceUnit.CM);

        if (dist > maxDistanceCm) {
            currentColor = None;
            return;
        }

        if (hsvValues[1] < minSaturation) {
            currentColor = None;
            return;
        }

        if (Math.abs(filteredHue - greenTargetHue) < hueTolerance) {
            currentColor = Green;
        }
        else if (Math.abs(filteredHue - purpleTargetHue) < hueTolerance) {
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
        telemetry.addData("Hue", hsvValues[0]);
        telemetry.addData("Saturation", hsvValues[1]);
        telemetry.addData("Value", hsvValues[2]);
        telemetry.addData("Green Target Hue", greenTargetHue);
        telemetry.addData("Purple Target Hue", purpleTargetHue);
        telemetry.addData("Detected Color", currentColor);
    }
}