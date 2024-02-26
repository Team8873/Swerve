package frc.robot.ui;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/** A class that represents a simple editable number on Shuffleboard */
public class SimpleNumber {
    private GenericEntry entry;

    /** Create a new SimpleNumber with the specified settings.
     * @param tab The Shuffleboard tab to place the widget on.
     * @param title The title of the widget.
     * @param position The position of the widget.
     * @param defaultValue The default value of the widget.
     */
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

    /** Get the current value associated with this widget.
     * @return The value currently associated with this widget, or 0 if it coould not be gotten.
     */
    public double get() {
        return entry.getDouble(0.0);
    }
}
