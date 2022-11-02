package org.firstinspires.ftc.teamcode.RecordTests.sensor.modernrobotics;

import org.firstinspires.ftc.teamcode.RecordTests.util.RollingAverage;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

/**
 * Reads the robot battery voltage
 */
public class Voltage {
    public final static int samples = 2000;
    VoltageSensor sensor;
    RollingAverage<Double> average;

    public Voltage(HardwareMap map) {
        sensor = map.voltageSensor.iterator().next();
        average = new RollingAverage<>(samples);
    }

    public void update() {
        average.addValue(getVoltageInstantaneous());
    }

    public double getVoltage() {
        return average.getAverage();
    }

    public double getVoltageInstantaneous() {
        return sensor.getVoltage();
    }
}
