package frc.lib.units;

import edu.wpi.first.wpilibj.DriverStation;

public class MeasurementRange<M extends Measurement> {
    private final M low, high;


    public MeasurementRange(M low, M high) {
        if (low.getCanonicalValue() > high.getCanonicalValue()) {
            DriverStation.reportWarning("MeasurementRange constructed with low > high", true);
        }
        this.low = low;
        this.high = high;
    }

    public boolean contains(M measurement) {
        return this.high.getCanonicalValue() > measurement.getCanonicalValue()
            && this.low.getCanonicalValue() < measurement.getCanonicalValue();
    }

    public M clampMeasurement(M measurement) {
        return Measurement.clamp(measurement, low, high);
    }
}
