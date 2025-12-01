package org.firstinspires.ftc.teamcode.Core.Hardware;


import static org.firstinspires.ftc.teamcode.Constants.Color.Green;
import static org.firstinspires.ftc.teamcode.Constants.Color.None;
import static org.firstinspires.ftc.teamcode.Constants.Color.Purple;
import static org.firstinspires.ftc.teamcode.Constants.Intake.ColorSensorConstants.GreenValuesHSV;
import static org.firstinspires.ftc.teamcode.Constants.Intake.ColorSensorConstants.PurpleValuesHSV;
import static org.firstinspires.ftc.teamcode.Constants.Intake.ColorSensorConstants.Treshold;
import static org.firstinspires.ftc.teamcode.Constants.Intake.ColorSensorConstants.currentColor;
import static org.firstinspires.ftc.teamcode.Constants.Intake.ColorSensorConstants.targetGreenRGB;
import static org.firstinspires.ftc.teamcode.Constants.Intake.ColorSensorConstants.targetPurpleRGB;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Core.Algorithms.LowPassFilter;

import java.util.Arrays;

@Config
public class HighSensor extends HighModule{
    RevColorSensorV3 sensor;
    private final LowPassFilter redFilter, blueFilter, greenFilter;
    double filterParameter = 0.8;
    float[] hsvValues = new float[3];
    float[] rgbValues = new float[3];
    double greenError = 0, purpleError = 0;

    public HighSensor(HardwareMap hardwareMap, String name) {
        sensor = hardwareMap.get(RevColorSensorV3.class, name);

        redFilter = new LowPassFilter(filterParameter, sensor.red());
        greenFilter = new LowPassFilter(filterParameter, sensor.green());
        blueFilter = new LowPassFilter(filterParameter, sensor.blue());
    }

    public double getDistance(DistanceUnit distanceUnit) {
        return sensor.getDistance(distanceUnit);
    }

    private void updateColor() {
        rgbValues[0] = (float) redFilter.getValue(sensor.red());
        rgbValues[1] = (float) greenFilter.getValue(sensor.green());
        rgbValues[2] = (float) blueFilter.getValue(sensor.blue());
    }

    private void updateSetColorHSV() {
        if (Math.abs(hsvValues[0] - GreenValuesHSV[0]) <= Treshold[0]) {
            currentColor = Green;
        } else if (Math.abs(hsvValues[0] - PurpleValuesHSV[0]) <= Treshold[0]) {
            currentColor = Purple;
        } else {
            currentColor = None;
        }
    }

    private void updateSetColorRGB() {
        if(greenError < purpleError && greenError <= 5){
            currentColor = Green;
        } else if(purpleError < greenError && purpleError <= 5){
            currentColor = Purple;
        } else {
            currentColor = None;
        }
    }

    @Override
    public void update() {
        updateColor();
        greenError = getError(rgbValues, targetGreenRGB);
        purpleError = getError(rgbValues, targetPurpleRGB);
        updateSetColorRGB();
    }

    public void updateHSV() {
        updateColor();
        updateSetColorHSV();
    }

    public Constants.Color getColor() {
        return currentColor;
    }

    public float[] RGB() {
        return rgbValues;
    }

    public float[] getHSVColorValues(){
        android.graphics.Color.RGBToHSV((int)rgbValues[0] * 8, (int)rgbValues[1] * 8, (int)rgbValues[2] * 8, hsvValues);
        return hsvValues;
    }

    private double getError(float r1, float r2, float g1, float g2, float b1, float b2){
        return Math.sqrt((r1-r2)*(r1-r2)+(g1-g2)*(g1-g2)+(b1-b2)*(b1-b2));
    }

    private double getError(float[] rgb1, float[] rgb2){
        return Math.sqrt((rgb1[0]-rgb2[0])*(rgb1[0]-rgb2[0])+(rgb1[1]-rgb2[1])*(rgb1[1]-rgb2[1])+(rgb1[2]-rgb2[2])*(rgb1[2]-rgb2[2]));
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Color in RGB", Arrays.toString(rgbValues));
        telemetry.addData("Color in Red", (sensor.red()));
        telemetry.addData("Color in Green", (sensor.green()));
        telemetry.addData("Color in Blue", (sensor.blue()));
        telemetry.addData("Color in HSV", Arrays.toString(this.hsvValues));
        telemetry.addData("Error Green", greenError);
        telemetry.addData("Target Green", Arrays.toString(targetGreenRGB));
        telemetry.addData("Error Purple", purpleError);
        telemetry.addData("Target Purple", Arrays.toString(targetPurpleRGB));
        telemetry.addData("Color known", this.getColor());
    }
}