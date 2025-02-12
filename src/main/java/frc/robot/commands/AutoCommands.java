package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.scoring.ScoringSuperstructure;
import frc.robot.subsystems.scoring.ScoringSuperstructureState;

import java.util.function.Supplier;

public class AutoCommands {
    private static final CommandSwerveDrivetrain swerve = CommandSwerveDrivetrain.getInstance();
    private static final ScoringSuperstructure superstructure = ScoringSuperstructure.getInstance();
    public static final Supplier<Command> oneSecondTimeout = () -> swerve.stop().withTimeout(1);
    //TODO: do these need to be suppliers
    public static final Supplier<Command> L4Score =
            () -> Commands.deadline(
                    Commands.sequence(
                            superstructure.setScoringState(ScoringSuperstructureState.L4),
                            superstructure.runScoringState().until(superstructure.isStateFinished),
                            superstructure.setScoringState(ScoringSuperstructureState.IDLE),
                            superstructure.runScoringState().until(superstructure.isStateFinished)
                    ),
                    swerve.stop()
            );
    public static final Supplier<Command> HPLoad =
            () -> Commands.deadline(
                    Commands.sequence(
                            superstructure.setScoringState(ScoringSuperstructureState.HP_LOADING),
                            superstructure.runScoringState().until(superstructure.isStateFinished),
                            superstructure.setScoringState(ScoringSuperstructureState.IDLE),
                            superstructure.runScoringState().until(superstructure.isStateFinished)
                    ),
                    swerve.stop()
            );
}
