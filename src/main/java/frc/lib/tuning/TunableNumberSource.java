package frc.lib.tuning;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;

public class TunableNumberSource implements NumberSource {
    private final String name;
    private final double defaultValue;
    private final List<Consumer<Double>> listeners = new ArrayList<>();
    private double lastValue = Double.NaN;

    public TunableNumberSource(String name, double defaultValue) {
        this.name = name;
        this.defaultValue = defaultValue;
        if (!SmartDashboard.containsKey(name)) {
            SmartDashboard.putNumber(name, this.defaultValue);
        }
    }

    @Override
    public String name() {
        return name;
    }

    @Override
    public double valueAsDouble() {
        return SmartDashboard.getNumber(name, defaultValue);
    }

    @Override
    public void update() {
        double value = valueAsDouble();
        if (value != lastValue) {
            lastValue = value;
            listeners.forEach(l -> l.accept(value));
        }
    }

    @Override
    public void addListener(Consumer<Double> listener) {
        listeners.add(listener);
    }
}
