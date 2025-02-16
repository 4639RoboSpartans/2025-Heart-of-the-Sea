package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drivetrain;
import frc.robot.subsystems.scoring.ScoringSuperstructure;
import frc.robot.subsystems.scoring.ScoringSuperstructureState;

import java.util.function.Supplier;

public class AutoCommands {
    private static final Drivetrain swerve = Drivetrain.getInstance();
    private static final ScoringSuperstructure superstructure = ScoringSuperstructure.getInstance();
    public static final Supplier<Command> oneSecondTimeout = () -> swerve.stop().withTimeout(1);
    public static final Supplier<Command> L4Score = () -> getScoringSuperstructureCommand(ScoringSuperstructureState.L4);
    public static final Supplier<Command> L3Score = () -> getScoringSuperstructureCommand(ScoringSuperstructureState.L3);
    public static final Supplier<Command> L2Score = () -> getScoringSuperstructureCommand(ScoringSuperstructureState.L2);
    public static final Supplier<Command> L1Score = () -> getScoringSuperstructureCommand(ScoringSuperstructureState.L1);
    public static final Supplier<Command> HPLoad = () -> getScoringSuperstructureCommand(ScoringSuperstructureState.HP_LOADING);

    private static Command getScoringSuperstructureCommand(ScoringSuperstructureState state) {
        return Commands.deadline(
            Commands.sequence(
                superstructure.setScoringState(state),
                superstructure.runScoringState().until(superstructure.isStateFinished),
                superstructure.setScoringState(ScoringSuperstructureState.IDLE),
                superstructure.runScoringState().until(superstructure.isStateFinished)
            ),
            swerve.stop()
        );
    }
}
