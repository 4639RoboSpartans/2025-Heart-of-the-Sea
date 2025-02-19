package frc.lib;

import com.google.errorprone.annotations.Immutable;

import edu.wpi.first.math.MathUtil;
import frc.robot.subsystems.scoring.constants.ScoringConstants;

import java.util.Arrays;
import java.util.function.UnaryOperator;

/**
 * Wraps a double as a proportion
 */
@Immutable
public final class Proportion {
    private final double value;

    private Proportion(double value) {
        this.value = value;
    }

    private static Proportion ofPrivate(Double d) throws InvalidProportionException{
        if(!d.equals(MathUtil.clamp(d, 0.0, 1.0)))
            throw new InvalidProportionException(d);
        else return new Proportion(d);
    }

    public static Proportion of(double d){
        try{
            return ofPrivate(d);
        }catch(InvalidProportionException e){
            System.err.println(e.getMessage());
            System.err.println(Arrays.toString(e.getStackTrace()));
            return new Proportion(MathUtil.clamp(d, 0.0, 1.0));
        }
    }

    public double getValue() {
        return value;
    }

    public Proportion plus(double d){
        return of(value + d);
    }

    public Proportion plus(Proportion p){
        return plus(p.value);
    }

    public Proportion minus(double d){
        return of(value - d);
    }

    public Proportion minus(Proportion p){
        return minus(p.value);
    }

    public Proportion times(double d){
        return of(value * d);
    }

    public Proportion times(Proportion p){
        return times(p.value);
    }

    public Proportion dividedBy(double d){
        return of(value / d);
    }

    public Proportion dividedBy(Proportion p){
        return dividedBy(p.value);
    }

    public Proportion exponentiatedBy(double d){
        return of(Math.pow(value, d));
    }

    public Proportion exponentiatedBy(Proportion p){
        return exponentiatedBy(p.value);
    }

    /**
     * Converts the Proportion to a unit and returns the double value in the provided unit.
     * 
     * @param unitConvertor how we convert the units. <p> Ex: <em>exampleProportion.toUnit(ScoringConstants.ElevatorConstants.ProportionToPosition::convert);<em>
     * @return
     */
    public double toUnit(UnaryOperator<Double> unitConvertor){
        return unitConvertor.apply(this.value);
    }

    public static class InvalidProportionException extends RuntimeException {

        public InvalidProportionException(double value) {
            super("Invalid Proportion: " + value + ". Proportions must be between 0 and 1. This proportion has been replaced with " + MathUtil.clamp(value, 0.0, 1.0));
        }
    }
}
