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

/** A simple double parameter store that persists data between reloads */
public class ParameterStore {
    private static ParameterStore instance;
    private HashMap<String, Double> store;

    /** Get the ParameterStore instance or create it and initialize is if it does not exist.
     * @return The ParameterStore instance.
     */
    private static ParameterStore getInstance() {
        if (instance == null) {
            instance = new ParameterStore();
            instance.initialize();
        }
        return instance;
    }

    /** Get the value associated with the given key, or associate that key with a default value if it is not in the store.
     * 
     * @param key The key to use.
     * @param defaultValue The default value to use if the key does not exist in the store.
     * @return The value in the store, or the default value if the key was not present in the store.
     */
    public static Double get(String key, Double defaultValue) {
        Double value = getInstance().store.putIfAbsent(key, defaultValue);
        if (value == null) value = defaultValue;
        return value;
    }

    /** Set the value of the specified key in the store.
     * 
     * @param key The key to use.
     * @param value The value to set it to.
     */
    public static void set(String key, Double value) {
        getInstance().store.put(key, value);
    }

    /** Initialize the parameter store from persistent storage. */
    private void initialize() {
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

    /** Save the current values in the parameter store to persistent storage. */
    public static void saveStore() {
        File dataFile = Paths
        .get(
            Filesystem.getOperatingDirectory().toString(),
            "team8873-saved.csv")
        .toFile();

        try (BufferedWriter writer = new BufferedWriter(new FileWriter(dataFile))) {
            getInstance().store.forEach((s, d) -> {
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
