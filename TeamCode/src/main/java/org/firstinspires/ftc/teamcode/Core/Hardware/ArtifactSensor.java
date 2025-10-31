package org.firstinspires.ftc.teamcode.Core.Hardware;


import static org.firstinspires.ftc.teamcode.Constants.Color.Green;
import static org.firstinspires.ftc.teamcode.Constants.Color.None;
import static org.firstinspires.ftc.teamcode.Constants.Color.Purple;
import static org.firstinspires.ftc.teamcode.Constants.GreenValuesHSV;
import static org.firstinspires.ftc.teamcode.Constants.PurpleValuesHSV;
import static org.firstinspires.ftc.teamcode.Constants.Treshold;
import static org.firstinspires.ftc.teamcode.Constants.currentColor;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Core.Algorithms.LowPassFilter;

import java.util.Arrays;

//todo rework this please
@Config
public class ArtifactSensor {
    ColorRangeSensor sensor;
    private final LowPassFilter redFilter, blueFilter, greenFilter;
    double filterParameter = 0.8;
    private int r, g, b;
    float[] hsvValues = new float[4];

    public ArtifactSensor(HardwareMap hardwareMap, String name) {
        sensor = hardwareMap.get(ColorRangeSensor.class, name);

        redFilter = new LowPassFilter(filterParameter, sensor.red());
        greenFilter = new LowPassFilter(filterParameter, sensor.green());
        blueFilter = new LowPassFilter(filterParameter, sensor.blue());
    }

    public double getDistance(DistanceUnit distanceUnit) {
        return sensor.getDistance(distanceUnit);
    }

    public void update() {
        r = (int) redFilter.getValue(sensor.red());
        g = (int) greenFilter.getValue(sensor.green());
        b = (int) blueFilter.getValue(sensor.blue());

        android.graphics.Color.RGBToHSV(r * 8, g * 8, b * 8, hsvValues);
        if (Math.abs(hsvValues[0] - GreenValuesHSV[0]) <= Treshold[0]) {
            currentColor = Green;
        } else if (Math.abs(hsvValues[0] - PurpleValuesHSV[0]) <= Treshold[0]) {
            currentColor = Purple;
        } else {
            currentColor = None;
        }
    }

    public Constants.Color getColor() {
        return currentColor;
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Color in RGB", Arrays.toString(this.RGB()));
        telemetry.addData("Color in HSV", Arrays.toString(this.hsvValues));
        telemetry.addData("Color known", this.getColor());
    }

    public double[] RGB() {
        return new double[]{r, g, b};
    }
    public float[] getHSVColorValues(){
        return hsvValues;
    }
}