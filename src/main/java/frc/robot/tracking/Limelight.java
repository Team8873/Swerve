package frc.robot.tracking;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
        limelight = NetworkTableInstance.getDefault().getTable("limelight");
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
        getInstance().priorityid.setInteger(id);
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
        SmartDashboard.putNumberArray("tpos", pos);
        return new FieldPos(pos[0], pos[1], pos[2], pos[3], pos[4], pos[5]);
    }

    public static Pose2d getRobotPos() {
        double[] pos = getInstance().limelight.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
        return new Pose2d(pos[0], pos[1], Rotation2d.fromDegrees(pos[5]));
    }

    public static void camModeDriver() {
        getInstance().limelight.getEntry("camMode").setDouble(0);
        getInstance().limelight.getEntry("ledMode").setDouble(0);
    }

    public static void camModeVision() {
        getInstance().limelight.getEntry("camMode").setDouble(0);
        getInstance().limelight.getEntry("ledMode").setDouble(0);
    }

    public static void camStreamSetup() {
        getInstance().limelight.getEntry("stream").setDouble(2);
    }

    public static record ScreenPos(double x, double y) {}
    public static record FieldPos(double x, double y, double z, double roll, double pitch, double yaw) {}
}
