package frc.robot;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class SliderFella {
    private GenericEntry entry;
    private double _default;

    public SliderFella(double defaultValue, String name, ShuffleboardTab tab) {
        _default = defaultValue;
        entry = tab 
            .add(name, defaultValue)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .getEntry();
    }

    public double get() {
        return entry.getDouble(_default);
    }
}
