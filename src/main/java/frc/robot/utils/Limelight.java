package frc.robot.utils;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/** A class that wraps low-level limelight functionality */
public class Limelight {
    private static Limelight instance;

    private NetworkTable limelight;

    private NetworkTableEntry tv;
    private NetworkTableEntry tx;
    private NetworkTableEntry ty;
    private NetworkTableEntry targetpose;
    private NetworkTableEntry priorityid;

    /** Construct a new Limelight instance. */
    private Limelight() {
        NetworkTableInstance.getDefault().getTable("limelight");
        tv = limelight.getEntry("tv");
        tx = limelight.getEntry("tx");
        ty = limelight.getEntry("ty");
        targetpose = limelight.getEntry("targetpose_robotspace");
        priorityid = limelight.getEntry("priorityid");
    }

    /** Get the current Limelight instance, or create it if it does not exist.
     * @return The current limelight instance.
     */
    private static Limelight getInstance() {
        if (instance == null) {
            instance = new Limelight();
        }
        return instance;
    }

    /** Get whether a target is currently detected.
     * @return Whether a target is currently detected.
     */
    public static boolean isTargetDetected() {
        return getInstance().tv.getDouble(0.0) == 1;
    }

    /** Set the priority targeting if.
     * @param id The id to target.
     */
    public static void setPriority(int id) {
        getInstance().priorityid.setDouble(id);
    }

    /** Get the position of the detected tag on the screen, or (0.0, 0.0) if a tag is not detected.
     * @return The position of the tag on the screen.
     */
    public static ScreenPos getTagScreenPos() {
        var ll = getInstance();
        return new ScreenPos(ll.tx.getDouble(0.0), ll.ty.getDouble(0.0));
    }

    /** Get the position of the tag relative to the robot, or a zeroed out position if a tag is not detected.
     * @return The position of the tag relative to the robot.
     */
    public static FieldPos getTagFieldPos() {
        double[] pos = getInstance().targetpose.getDoubleArray(new double[6]);
        return new FieldPos(pos[0], pos[1], pos[2], pos[3], pos[4], pos[5]);
    }

    public static record ScreenPos(double x, double y) {}
    public static record FieldPos(double x, double y, double z, double roll, double pitch, double yaw) {}
}
