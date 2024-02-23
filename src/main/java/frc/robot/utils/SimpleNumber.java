package frc.robot.utils;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class SimpleNumber {
    private GenericEntry entry;

    public SimpleNumber(ShuffleboardTab tab, String title, Position position, double defaultValue) {
        var entry = tab
            .add(title, defaultValue)
            .withWidget(BuiltInWidgets.kTextView)
            .withSize(1, 1)
            .withPosition(position.column(), position.row())
            .getEntry();

        this.entry = entry;

        entry.setDouble(defaultValue);
    }

    public double get() {
        return entry.getDouble(0.0);
    }
}
