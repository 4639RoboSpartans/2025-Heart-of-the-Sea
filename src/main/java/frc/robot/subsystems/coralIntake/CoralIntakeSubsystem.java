package frc.robot.subsystems.coralIntake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.Objects;

public abstract class CoralIntakeSubsystem extends SubsystemBase {
    private static CoralIntakeSubsystem instance;

    public static CoralIntakeSubsystem getInstance() {
        return Objects.requireNonNullElseGet(instance, ConcreteCoralIntakeSubsystem::new);
    }

    protected abstract void setIntakeState(CoralIntakeState intakeState);

    public Command setIntakeStateCommand(CoralIntakeState state) {
        return Commands.runOnce(
                () -> setIntakeState(state)
        );
    }

    public abstract Trigger atRequestedState();
}
