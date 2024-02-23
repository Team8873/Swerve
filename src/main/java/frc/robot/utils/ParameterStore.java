package frc.robot.utils;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Paths;
import java.util.HashMap;

import edu.wpi.first.wpilibj.Filesystem;

public class ParameterStore {
    private static HashMap<String, Double> store;

    public static Double get(String key, Double defaultValue) {
        Double value = store.putIfAbsent(key, defaultValue);
        if (value == null) value = defaultValue;
        return value;
    }

    public static void set(String key, Double value) {
        store.put(key, value);
    }

    public static void initialize() {
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
