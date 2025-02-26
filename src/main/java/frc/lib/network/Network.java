package frc.lib.network;

import java.util.HashMap;
import java.util.Map;
import java.util.Objects;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Network {
    //singleton
    private static volatile Network instance;

    static synchronized Network getInstance() {
        return Objects.requireNonNullElseGet(instance, () -> instance = new Network());
    }

    private final NetworkTableInstance networkTableInstance = NetworkTableInstance.getDefault();

    private final Map<String, Table> tableCache = new HashMap<>();

    public static Table getTable(String tableName) {
        return getInstance().tableCache.computeIfAbsent(tableName, key -> new Table(tableName, getInstance().networkTableInstance.getTable(key)));
    }

    public static void printAllEntriesFromTable(String tableName){
        getTable(tableName).getAllEntries().forEach(
            (key, value) -> SmartDashboard.putString(tableName + ":" + key, value)
        );
    }
}
