package frc.lib.units;

import java.util.function.DoubleFunction;
import java.util.function.Supplier;

public abstract class Measurement {
    public abstract double getCanonicalValue();
    protected abstract void setCanonicalValue(double value);

    /** Must return an instance of itself */
    protected abstract Measurement copy();

    protected static <M extends Measurement> M create(Supplier<M> constructor, double value) {
        M measurement = constructor.get();
        measurement.setCanonicalValue(value);
        return measurement;
    }

    @SuppressWarnings("unchecked")
    private static <M extends Measurement> M copy(M measurement) {
        return (M) measurement.copy();
    }

    public static <M extends Measurement> MeasurementOffset<M> sub(M a, M b) {
        return MeasurementOffset.createFromSubtraction(a, b);
    }

    public static <M extends Measurement> MeasurementOffset<M> createOffset(DoubleFunction<M> conversion, double amount) {
        M measurement = conversion.apply(amount);
        M zero = conversion.apply(0);
        // Assuming that conversion(x).canonicalValue = m * x + b
        // is a linear function of x, the offset we desire is just
        // m * amount (because we must consider only the slope), which
        // is equal to conversion(amount) - conversion(0)
        // = (m * amount + b) - (m * 0 + b) = m * amount
        return sub(measurement, zero);
    }

    public static <M extends Measurement> MeasurementOffset<M> zeroOffset() {
        return new MeasurementOffset<>(0);
    }

    public static <M extends Measurement> M add(M measurement, MeasurementOffset<M> offset) {
        return offset.addTo(copy(measurement));
    }

    public static <M extends Measurement> M sub(M measurement, MeasurementOffset<M> offset) {
        return add(measurement, offset.times(-1));
    }

    public static <M extends Measurement> M max(M a, M b) {
        return a.getCanonicalValue() > b.getCanonicalValue() ? a : b;
    }

    public static <M extends Measurement> MeasurementOffset<M> max(MeasurementOffset<M> a, MeasurementOffset<M> b) {
        return a.getCanonicalValue() > b.getCanonicalValue() ? a : b;
    }

    public static <M extends Measurement> M min(M a, M b) {
        return a.getCanonicalValue() < b.getCanonicalValue() ? a : b;
    }

    public static <M extends Measurement> MeasurementOffset<M> min(MeasurementOffset<M> a, MeasurementOffset<M> b) {
        return a.getCanonicalValue() < b.getCanonicalValue() ? a : b;
    }

    public static <M extends Measurement> MeasurementRange<M> withTolerance(M measurement, MeasurementOffset<M> offset) {
        return new MeasurementRange<>(
            sub(measurement, offset),
            add(measurement, offset)
        );
    }

    public static <M extends Measurement> M clamp(M measurement, M low, M high) {
        return Measurement.max(Measurement.min(measurement, high), low);
    }

    public static <M extends Measurement> MeasurementOffset<M> clamp(MeasurementOffset<M> measurement, MeasurementOffset<M> low, MeasurementOffset<M> high) {
        return Measurement.max(Measurement.min(measurement, high), low);
    }

}
