package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystemManager.Subsystems;
import frc.robot.subsystems.drive.AbstractSwerveDrivetrain;
import frc.robot.subsystems.scoring.ScoringSuperstructure;
import frc.robot.subsystems.scoring.ScoringSuperstructureAction;

import java.util.function.BooleanSupplier;
import java.util.function.Function;
import java.util.function.Supplier;

public class AutoCommands {
    private static final AbstractSwerveDrivetrain swerve = Subsystems.drivetrain();
    private static final ScoringSuperstructure superstructure = Subsystems.scoringSuperstructure();
    public static final Supplier<Command> SwerveStop = swerve::stop;
    public static final Supplier<Command> L4Score = () -> getScoringSuperstructureCommand(ScoringSuperstructureAction.SCORE_L4_CORAL);
    public static final Supplier<Command> L3Score = () -> getScoringSuperstructureCommand(ScoringSuperstructureAction.SCORE_L3_CORAL);
    public static final Supplier<Command> L2Score = () -> getScoringSuperstructureCommand(ScoringSuperstructureAction.SCORE_L2_CORAL);
    public static final Supplier<Command> L1Score = () -> getScoringSuperstructureCommand(ScoringSuperstructureAction.SCORE_L1_CORAL);
    public static final Supplier<Command> HPLoad = () -> getScoringSuperstructureCommand(ScoringSuperstructureAction.INTAKE_FROM_HP);
    public static final Supplier<Command> HPLoad_Lower = () -> getScoringSuperstructureCommand(ScoringSuperstructureAction.INTAKE_FROM_HP_LOWER)
            .until(superstructure::hasCoral);

    public static final Function<Boolean, Command> setAutoOuttake = shouldOuttake -> superstructure.setAutoOuttake(shouldOuttake);

    public static final BooleanSupplier hasCoral = superstructure::hasCoral;

    private static Command getScoringSuperstructureCommand(ScoringSuperstructureAction action) {
        return superstructure.setAction(action);
    }
}
