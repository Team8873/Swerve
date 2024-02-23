package frc.robot.utils;

import java.util.ArrayList;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class SimpleButton {
    private static ArrayList<SimpleButton> instances;
    private GenericEntry entry;
    private Runnable callback;

    private SimpleButton(GenericEntry entry, Runnable callback) {
        this.entry = entry;
        this.callback = callback;

        entry.setBoolean(false);
    }

    private void update() {
        if (entry.getBoolean(false)) {
            entry.setBoolean(false);
            callback.run();
        }
    }

    public static void updateAll() {
        for (var button : instances) {
            button.update();
        }
    }

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
