package frc.robot.utils;

import com.revrobotics.*;
import com.revrobotics.Rev2mDistanceSensor.Port;

public class DistanceSensor {
    private Rev2mDistanceSensor sensor;

    private static DistanceSensor instance;

    private DistanceSensor() {
        sensor = new Rev2mDistanceSensor(Port.kOnboard);
    }

    private static DistanceSensor getInstance() {
        if (instance == null) {
            instance = new DistanceSensor();
        }
        return instance;
    }

    public static boolean isDetecting() {
        return getInstance().sensor.isRangeValid();
    }

    public static double distance() {
        return getInstance().sensor.getRange();
    }

    public static void init() {
        getInstance().sensor.setAutomaticMode(true);
    }

    public static void deinit() {
        getInstance().sensor.setAutomaticMode(false);
    }
}
