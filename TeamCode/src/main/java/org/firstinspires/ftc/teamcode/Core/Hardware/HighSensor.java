package org.firstinspires.ftc.teamcode.Core.Hardware;

import static org.firstinspires.ftc.teamcode.Constants.Color.Green;
import static org.firstinspires.ftc.teamcode.Constants.Color.None;
import static org.firstinspires.ftc.teamcode.Constants.Color.Purple;
import static org.firstinspires.ftc.teamcode.Constants.Intake.ColorSensorConstants.GreenValuesHSV;
import static org.firstinspires.ftc.teamcode.Constants.Intake.ColorSensorConstants.PurpleValuesHSV;
import static org.firstinspires.ftc.teamcode.Constants.Intake.ColorSensorConstants.Treshold;
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
    public static double minSaturation = 0.2;
    private volatile float[] hsvValues = new float[3];
    private Thread colorThread;
    private final int burstSize = 9;

    public HighSensor(HardwareMap hardwareMap, String name) {
        sensor = hardwareMap.get(RevColorSensorV3.class, name);
        sensor.setGain(25.0f);
    }

    public double getDistance(DistanceUnit distanceUnit) {
        return sensor.getDistance(distanceUnit);
    }

    private void processBatch() {
        float[] avgHsv = new float[3];
        int greenCount = 0;
        int purpleCount = 0;
        int noneCount = 0;
        float[] tempHsv = new float[3];

        for (int i = 0; i < burstSize; i++) {
            NormalizedRGBA colors = sensor.getNormalizedColors();
            Color.colorToHSV(colors.toColor(), tempHsv);

            avgHsv[0] += tempHsv[0];
            avgHsv[1] += tempHsv[1];
            avgHsv[2] += tempHsv[2];

            if (tempHsv[1] < minSaturation) {
                noneCount++;
            } else {
                float hue = tempHsv[0];
                float threshold = Treshold[0];

                if (Math.abs(hue - GreenValuesHSV[0]) <= threshold && hue <=200) {
                    greenCount++;
                } else if (Math.abs(hue - PurpleValuesHSV[0]) <= threshold && hue >= 185) {
                    purpleCount++;
                } else {
                    noneCount++;
                }
            }
        }

        avgHsv[0] /= burstSize;
        avgHsv[1] /= burstSize;
        avgHsv[2] /= burstSize;
        hsvValues = avgHsv;

        if (greenCount > purpleCount && greenCount > noneCount) {
            currentColor = Green;
        } else if (purpleCount > greenCount && purpleCount > noneCount) {
            currentColor = Purple;
        } else {
            currentColor = None;
        }
    }

    @Override
    public void update() {
        if (colorThread == null || !colorThread.isAlive()) {
            colorThread = new Thread(this::processBatch);
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
        telemetry.addData("Distance (CM)", sensor.getDistance(DistanceUnit.CM));
        telemetry.addData("HSV", Arrays.toString(hsvValues));
        telemetry.addData("Hue", hsvValues[0]);
        telemetry.addData("Saturation", hsvValues[1]);
        telemetry.addData("Value", hsvValues[2]);
        telemetry.addData("Green Target Hue", GreenValuesHSV[0]);
        telemetry.addData("Purple Target Hue", PurpleValuesHSV[0]);
        telemetry.addData("Detected Color", currentColor);
    }
}