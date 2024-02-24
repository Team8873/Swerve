package frc.robot.utils;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight {
    private static Limelight instance;

    private NetworkTable limelight;

    private NetworkTableEntry tv;
    private NetworkTableEntry tx;
    private NetworkTableEntry ty;
    private NetworkTableEntry targetpose;
    private NetworkTableEntry priorityid;

    private Limelight() {
        NetworkTableInstance.getDefault().getTable("limelight");
        tv = limelight.getEntry("tv");
        tx = limelight.getEntry("tx");
        ty = limelight.getEntry("ty");
        targetpose = limelight.getEntry("targetpose_robotspace");
        priorityid = limelight.getEntry("priorityid");
    }

    private static Limelight getInstance() {
        if (instance == null) {
            instance = new Limelight();
        }
        return instance;
    }

    public static boolean isTargetDetected() {
        return getInstance().tv.getDouble(0.0) == 1;
    }

    public static void setPriority(int id) {
        getInstance().priorityid.setDouble(id);
    }

    public static ScreenPos getTagScreenPos() {
        var ll = getInstance();
        return new ScreenPos(ll.tx.getDouble(0.0), ll.ty.getDouble(0.0));
    }

    public static FieldPos getTagFieldPos() {
        double[] pos = getInstance().targetpose.getDoubleArray(new double[6]);
        return new FieldPos(pos[0], pos[1], pos[2], pos[3], pos[4], pos[5]);
    }

    public static record ScreenPos(double x, double y) {}
    public static record FieldPos(double x, double y, double z, double roll, double pitch, double yaw) {}
}
