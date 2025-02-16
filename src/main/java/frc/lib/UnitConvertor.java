package frc.lib;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.measure.Distance;

import java.util.function.Function;

@SuppressWarnings("unused")
public interface UnitConvertor<A, B> {
    /**
     * Performs the forward unit conversion
     */
    B convert(A amountA);

    /**
     * Performs the inverse unit conversion
     */
    A convertBackwards(B amountB);

    /**
     * Equivalent to {@link UnitConvertor#compose UnitConvertor.compose(this, next)}.
     */
    default <C> UnitConvertor<A, C> then(UnitConvertor<B, C> next) {
        return compose(this, next);
    }

    /**
     * Equivalent to {@link UnitConvertor#invert UnitConvertor.invert(this)}.
     */
    default UnitConvertor<B, A> inverted() {
        return invert(this);
    }

    /**
     * Returns the inverse of a unit convertor, swapping {@link UnitConvertor#convert} and
     * {@link UnitConvertor#convertBackwards}
     */
    static <A, B> UnitConvertor<B, A> invert(UnitConvertor<A, B> convertor) {
        return new UnitConvertor<>() {
            @Override
            public B convertBackwards(A amountB) {
                return convertor.convert(amountB);
            }

            @Override
            public A convert(B amountA) {
                return convertor.convertBackwards(amountA);
            }
        };
    }

    /**
     * Composes unit convertors.
     */
    static <A, B, C> UnitConvertor<A, C> compose(UnitConvertor<A, B> convertor1, UnitConvertor<B, C> convertor2) {
        return new UnitConvertor<>() {
            @Override
            public C convert(A amountA) {
                return convertor2.convert(convertor1.convert(amountA));
            }

            @Override
            public A convertBackwards(C amountB) {
                return convertor1.convertBackwards(convertor2.convertBackwards(amountB));
            }
        };
    }

    /**
     * Creates a unit converter between units that have a linear relationship
     */
    static UnitConvertor<Double, Double> linear(double factor) {
        return linear(factor, 0, false);
    }

    /**
     * Creates a unit converter between units that have a linear relationship, with an offset
     *
     * @param applyOffsetFirst If true, {@link UnitConvertor#convert convert(x)} returns
     *                         (x + offset) * factor, otherwise, it returns
     *                         (x * factor) + offset
     */
    static UnitConvertor<Double, Double> linear(double factor, double offset, boolean applyOffsetFirst) {
        double actualOffset = applyOffsetFirst ? offset * factor : offset;

        return new UnitConvertor<>() {
            @Override
            public Double convert(Double amountA) {
                return amountA * factor + actualOffset;
            }

            @Override
            public Double convertBackwards(Double amountB) {
                return (amountB - actualOffset) / factor;
            }
        };
    }

    /**
     * Creates a unit convertor that linearly scales a range (minA, maxA) to a range (minB, maxB).
     */
    static UnitConvertor<Double, Double> linearConvertingRange(double minA, double maxA, double minB, double maxB) {
        double factor = (maxB - minB) / (maxA - minA);
        return linear(
            factor,
            minB - factor * minA,
            false
        );
    }

    static <A, B> UnitConvertor<A, B> create(Function<A, B> forwards, Function<B, A> backwards) {
        return new UnitConvertor<>() {
            @Override
            public B convert(A amountA) {
                return forwards.apply(amountA);
            }

            @Override
            public A convertBackwards(B amountB) {
                return backwards.apply(amountB);
            }
        };
    }

    static UnitConvertor<Double, Distance> toDistance(DistanceUnit unit) {
        return create(unit::of, distance -> distance.in(unit));
    }

    static UnitConvertor<Double, Rotation2d> radiansToRotation2d() {
        return create(Rotation2d::fromRadians, Rotation2d::getRadians);
    }
}
