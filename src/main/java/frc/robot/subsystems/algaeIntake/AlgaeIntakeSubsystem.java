package frc.robot.subsystems.algaeIntake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Objects;
import java.util.function.DoubleSupplier;

public abstract class AlgaeIntakeSubsystem extends SubsystemBase {
    private static AlgaeIntakeSubsystem instance;

    public static AlgaeIntakeSubsystem getInstance() {
        return instance = Objects.requireNonNullElseGet(instance, ConcreteAlgaeIntakeSubsystem::new);
    }

    protected abstract void setPivotSpeed(double percent);

    protected abstract void setIntakeSpeed(double percent);

    public Command test(DoubleSupplier pivot_speed_percent, DoubleSupplier intake_speed_percent) {
        return run(() -> {
            setPivotSpeed(pivot_speed_percent.getAsDouble());
            setIntakeSpeed(intake_speed_percent.getAsDouble());
        }).finallyDo(() -> {
            setPivotSpeed(0);
            setIntakeSpeed(0);
        });
    }
}
