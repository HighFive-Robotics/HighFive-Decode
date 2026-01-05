package org.firstinspires.ftc.teamcode.Core.Hardware;

import static org.firstinspires.ftc.teamcode.Constants.Color.Green;
import static org.firstinspires.ftc.teamcode.Constants.Color.None;
import static org.firstinspires.ftc.teamcode.Constants.Color.Purple;
import static org.firstinspires.ftc.teamcode.Constants.Intake.ColorSensorConstants.GreenValuesHSV;
import static org.firstinspires.ftc.teamcode.Constants.Intake.ColorSensorConstants.PurpleValuesHSV;
import static org.firstinspires.ftc.teamcode.Constants.Intake.ColorSensorConstants.currentColor;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Constants;

import java.util.Arrays;

@Config
public class HighSensor extends HighModule {
    RevColorSensorV3 sensor;

    public static double minSaturation = 0.15;

    public static double minValue = 0.08;

    private volatile float[] hsvValues = new float[3];
    private Thread colorThread;
    private final int burstSize = 9;
    public static float sensorGain = 29.0f;
    public HighSensor(HardwareMap hardwareMap, String name) {
        sensor = hardwareMap.get(RevColorSensorV3.class, name);
        sensor.setGain(sensorGain);
    }

    public double getDistance(DistanceUnit distanceUnit) {
        return sensor.getDistance(distanceUnit);
    }

    public boolean isInReach(double distance){
        return sensor.getDistance(DistanceUnit.CM) <= distance;
    }

    private void processBatch() {
        float sumR = 0;
        float sumG = 0;
        float sumB = 0;

        for (int i = 0; i < burstSize; i++) {
            NormalizedRGBA colors = sensor.getNormalizedColors();

            sumR += colors.red;
            sumG += colors.green;
            sumB += colors.blue;
        }

        int r = (int) ((sumR / burstSize) * 255);
        int g = (int) ((sumG / burstSize) * 255);
        int b = (int) ((sumB / burstSize) * 255);

        r = Math.min(255, Math.max(0, r));
        g = Math.min(255, Math.max(0, g));
        b = Math.min(255, Math.max(0, b));

        Color.RGBToHSV(r, g, b, hsvValues);

        float hue = hsvValues[0];
        float sat = hsvValues[1];
        float val = hsvValues[2];

        if (sat < minSaturation || val < minValue) {
            currentColor = None;
            return;
        }

        float distToGreen = getAngularDistance(hue, GreenValuesHSV[0]);
        float distToPurple = getAngularDistance(hue, PurpleValuesHSV[0]);

        if (distToGreen < distToPurple) {
            currentColor = Green;
        } else {
            currentColor = Purple;
        }
    }

    private float getAngularDistance(float h1, float h2) {
        float diff = Math.abs(h1 - h2);
        return Math.min(diff, 360 - diff);
    }

    @Override
    public void update() {
        if (colorThread == null || !colorThread.isAlive()) {
            colorThread = new Thread(this::processBatch);
            colorThread.setPriority(Thread.MAX_PRIORITY);
            colorThread.start();
        }
    }

    public Constants.Color getColor() {
        return currentColor;
    }

    public float[] getHSVColorValues() {
        return hsvValues;
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Dist (CM)", String.format("%.2f", sensor.getDistance(DistanceUnit.CM)));
        telemetry.addData("HSV", String.format("%.1f, %.2f, %.2f", hsvValues[0], hsvValues[1], hsvValues[2]));

        float distG = getAngularDistance(hsvValues[0], GreenValuesHSV[0]);
        float distP = getAngularDistance(hsvValues[0], PurpleValuesHSV[0]);

        telemetry.addData("Delta Green", String.format("%.1f", distG));
        telemetry.addData("Delta Purple", String.format("%.1f", distP));
        telemetry.addData("Result", currentColor);
    }
}