package frc.robot.subsystems.coralIntake;
import frc.robot.subsystems.coralIntake.constants.CoralIntakeConstants.*;

public enum CoralIntakeState {
    INTAKING(0, 0.5),
    OUTTAKING(0, -0.5),
    EXTENDED(0, 0),
    IDLE(1, 0);

    private final double positionPercentage;
    public final double intakeSpeed;

    CoralIntakeState(double positionPercentage, double intakeSpeed) {
        this.positionPercentage = positionPercentage;
        this.intakeSpeed = intakeSpeed;
    }

    public double getPivotAbsolutePosition() {
        return PivotConstants.downPosition + positionPercentage * PivotConstants.positionDiff;
    }
}
