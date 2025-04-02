package frc.lib.units;

import frc.lib.annotation.PackagePrivate;

public class MeasurementOffset<M extends Measurement> {
    private final double canonicalOffset;

    @PackagePrivate
    MeasurementOffset(double canonicalOffset) {
        this.canonicalOffset = canonicalOffset;
    }

    @PackagePrivate
    static <M extends Measurement> MeasurementOffset<M> createFromSubtraction(M measurement1, M measurement2) {
        return new MeasurementOffset<>(measurement1.getCanonicalValue() - measurement2.getCanonicalValue());
    }

    @PackagePrivate
    M addTo(M measurement) {
        measurement.setCanonicalValue(measurement.getCanonicalValue() + canonicalOffset);
        return measurement;
    }

    public MeasurementOffset<M> times(double factor) {
        return new MeasurementOffset<>(canonicalOffset * factor);
    }

    public MeasurementOffset<M> plus(MeasurementOffset<M> offset) {
        return new MeasurementOffset<>(canonicalOffset + offset.canonicalOffset);
    }

    public MeasurementOffset<M> minus(MeasurementOffset<M> offset) {
        return new MeasurementOffset<>(canonicalOffset - offset.canonicalOffset);
    }

    public double getCanonicalValue() {
        return canonicalOffset;
    }
}
