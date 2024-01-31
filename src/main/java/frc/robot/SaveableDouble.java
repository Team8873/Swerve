package frc.robot;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Paths;
import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.Filesystem;

public class SaveableDouble {
    private static Map<String, Double> store;

    private final String key;
    private final Double defaultValue;
    private Double currentValue;

    public SaveableDouble(String key, Double defaultValue) {
        if (store == null) {
            initializeStore();
        }

        this.key = key;
        this.defaultValue = defaultValue;

        this.currentValue = store.putIfAbsent(key, defaultValue);
        if (this.currentValue == null) {
            this.currentValue = defaultValue;
        }
    }

    public Double get() {
        return currentValue;
    }

    public void set(Double value) {
        this.currentValue = value;
        store.put(this.key, value);
    }

    public void useDefault() {
        this.currentValue = defaultValue;
    }

    private static void initializeStore() {
        store = new HashMap<>();

        File dataFile = Paths
        .get(
            Filesystem.getOperatingDirectory().toString(),
            "team8873-saved.csv")
        .toFile();

        try (BufferedReader reader = new BufferedReader(new FileReader(dataFile))) {
            String line = reader.readLine();
            while (line != null) {
                String[] pieces = line.split(",");
                line = reader.readLine();
                store.put(pieces[0], Double.parseDouble(pieces[1]));
            }
        }
        // Ignore if the file cannot be found
        catch (FileNotFoundException e) {}
        catch (IOException e) {
            System.out.println("Failed to initialize store.");
            e.printStackTrace();
        }
    }

    public static void saveStore() {
        File dataFile = Paths
        .get(
            Filesystem.getOperatingDirectory().toString(),
            "team8873-saved.csv")
        .toFile();

        try (BufferedWriter writer = new BufferedWriter(new FileWriter(dataFile))) {
            store.forEach((s, d) -> {
                try {
                    writer.write(s + "," + d + "\n");
                }
                catch (Exception e) {
                    System.out.println("Failed to save store.");
                    e.printStackTrace();
                }
            });
        }
        catch (IOException e) {
            System.out.println("Failed to save store.");
            e.printStackTrace();
        }
    }
}
