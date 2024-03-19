package frc.robot.utils;

import com.revrobotics.*;
import com.revrobotics.Rev2mDistanceSensor.Port;

/** A thin wrapper around the REV 2 meter distance sensor. */
public class DistanceSensor {
    private Rev2mDistanceSensor sensor;

    private static DistanceSensor instance;
    
    /** Construct a new instance of the distance sensor. */
    private DistanceSensor() {
        sensor = new Rev2mDistanceSensor(Port.kOnboard);
    }

    /** Get the current instance of the distance sensor, or create it if it does not exist.
     * @return The current instance.
     */
    private static DistanceSensor getInstance() {
        if (instance == null) {
            instance = new DistanceSensor();
        }
        return instance;
    }

    /** Check if the distance sensor is detecting an object.
     * @return Whether the distance sensor is detecting an object.
     */
    public static boolean isDetecting() {
        return getInstance().sensor.isRangeValid();
    }
    
    /** Get the distance from the sensor to the object it is detecting.
     * @return The distance to the object.
     */
    public static double distance() {
        return getInstance().sensor.getRange();
    }
    
    /** Initialize the distance sensor. */
    public static void init() {
        getInstance().sensor.setAutomaticMode(true);
    }

    /** Deinitialize the distance sensor. */
    public static void deinit() {
        getInstance().sensor.setAutomaticMode(false);
    }
}
