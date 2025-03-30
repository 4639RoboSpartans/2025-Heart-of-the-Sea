package frc.lib.persist;

import edu.wpi.first.networktables.*;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * Uses NetworkTable persistence to record a value that is saved to the NetworkTables server.
 * When code is deployed, the value is either read from NetworkTables or given a default value.
 * As changes are made, those changes are updated both to the local variable and the NetworkTables topic.
 * <p></p>
 * It is expected that all input comes from within the robot code itself; if any changes are made using
 * another accessor of NetworkTables such as Elastic, they will not automatically be reflected unless the {@link PersistingDouble#update()}
 * method is called.
 */
public class PersistingDouble implements DoubleSupplier, Supplier<Double>{
    public static final NetworkTable TABLE = NetworkTableInstance.getDefault().getTable("persist");
    private final NetworkTableEntry entry;
    private double defaultValue = 0;
    private double value;

    public PersistingDouble(String name) {
        TABLE.getDoubleTopic(name).setPersistent(true);
        entry = TABLE.getEntry(name);
        value = entry.getDouble(defaultValue);
    }

    public PersistingDouble withDefaultValue(double defaultValue) {
        this.defaultValue = defaultValue;
        value = entry.getDouble(defaultValue);
        return this;
    }

    @Override
    public double getAsDouble() {
        return value;
    }

    @Override
    public Double get() {
        return getAsDouble();
    }

    public void setValue(double value) {
        this.value = value;
        entry.setDouble(value);
    }

    public void update(){
        value = entry.getDouble(defaultValue);
    }
}
