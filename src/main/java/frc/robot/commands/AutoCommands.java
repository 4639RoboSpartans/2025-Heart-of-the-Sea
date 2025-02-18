package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.drive.AbstractSwerveDrivetrain;
import frc.robot.subsystems.scoring.ScoringSuperstructure;
import frc.robot.subsystems.scoring.ScoringSuperstructureAction;

import java.util.function.Supplier;

public class AutoCommands {
    private static final AbstractSwerveDrivetrain swerve = SubsystemManager.getInstance().getDrivetrain();
    private static final ScoringSuperstructure superstructure = SubsystemManager.getInstance().getScoringSuperstructure();
    public static final Supplier<Command> oneSecondTimeout = () -> swerve.stop().withTimeout(1);
    public static final Supplier<Command> L4Score = () -> getScoringSuperstructureCommand(ScoringSuperstructureAction.SCORE_L4_CORAL);
    public static final Supplier<Command> L3Score = () -> getScoringSuperstructureCommand(ScoringSuperstructureAction.SCORE_L3_CORAL);
    public static final Supplier<Command> L2Score = () -> getScoringSuperstructureCommand(ScoringSuperstructureAction.SCORE_L2_CORAL);
    public static final Supplier<Command> L1Score = () -> getScoringSuperstructureCommand(ScoringSuperstructureAction.SCORE_L1_CORAL);
    public static final Supplier<Command> HPLoad = () -> getScoringSuperstructureCommand(ScoringSuperstructureAction.INTAKE_FROM_HP);

    private static Command getScoringSuperstructureCommand(ScoringSuperstructureAction state) {
        return Commands.deadline(
            Commands.sequence(
                superstructure.setScoringState(state),
                superstructure.runScoringState().until(superstructure.isStateFinished),
                superstructure.setScoringState(ScoringSuperstructureAction.IDLE),
                superstructure.runScoringState().until(superstructure.isStateFinished)
            ),
            swerve.stop()
        );
    }
}
