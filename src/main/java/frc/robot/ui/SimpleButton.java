package frc.robot.ui;

import java.util.ArrayList;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/** A class that manages simple push buttons on Shuffleboard */
public class SimpleButton {
    private static ArrayList<SimpleButton> instances = new ArrayList<>();
    private GenericEntry entry;
    private Runnable callback;

    /** Create a new SimpleButton with the given entry and callback.
     * @param entry The entry associated with the button.
     * @param callback The callback to run when the button is pressed.
     */
    private SimpleButton(GenericEntry entry, Runnable callback) {
        this.entry = entry;
        this.callback = callback;

        entry.setBoolean(false);
    }

    /** Check the button's state and run the callback if necessary */
    private void update() {
        if (entry.getBoolean(false)) {
            entry.setBoolean(false);
            callback.run();
        }
    }

    /** Check for any button presses and call any callbacks that need to be called */
    public static void updateAll() {
        for (var button : instances) {
            button.update();
        }
    }

    /** Create a new SimpleButton with the specified settings.
     * @param tab The Shuffleboard tab to place the button on.
     * @param title The title of the button.
     * @param position The position of the button.
     * @param callback A callback to run when the button is pressed.
     */
    public static void createButton(ShuffleboardTab tab, String title, Position position, Runnable callback) {
        var entry = tab
            .add(title, false)
            .withWidget(BuiltInWidgets.kToggleButton)
            .withSize(1, 1)
            .withPosition(position.column(), position.row())
            .getEntry();

        instances.add(new SimpleButton(entry, callback));
    }
}
