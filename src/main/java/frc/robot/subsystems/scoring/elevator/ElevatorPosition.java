package frc.robot.subsystems.scoring.elevator;

import edu.wpi.first.units.measure.Distance;
import frc.lib.units.Measurement;

import static frc.robot.subsystems.scoring.constants.ScoringConstants.ElevatorConstants.ProportionToHeight;
import static frc.robot.subsystems.scoring.constants.ScoringConstants.ElevatorConstants.ProportionToPosition;

public class ElevatorPosition extends Measurement {
    private double valueAsProportion;

    @Override
    public double getCanonicalValue() {
        return valueAsProportion;
    }

    @Override
    protected void setCanonicalValue(double value) {
        valueAsProportion = value;
    }

    @Override
    protected Measurement copy() {
        return Measurement.create(ElevatorPosition::new, getCanonicalValue());
    }

    public static ElevatorPosition fromProportion(double proportion) {
        return Measurement.create(ElevatorPosition::new, proportion);
    }

    public static ElevatorPosition fromMotorPosition(double position) {
        return Measurement.create(ElevatorPosition::new, ProportionToPosition.convertBackwards(position));
    }

    public static ElevatorPosition fromHeight(Distance height) {
        return Measurement.create(ElevatorPosition::new, ProportionToHeight.convertBackwards(height));
    }

    public double getProportion() {
        return this.valueAsProportion;
    }

    public double getMotorPosition() {
        return ProportionToPosition.convert(getProportion());
    }

    public Distance getHeight() {
        return ProportionToHeight.convert(getProportion());
    }
}
