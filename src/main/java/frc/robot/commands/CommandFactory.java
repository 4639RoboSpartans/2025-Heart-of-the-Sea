package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.Controls;
import frc.robot.constants.FieldConstants.TargetPositions.Direction;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.climber.AbstractClimberSubsystem;
import frc.robot.subsystems.climber.ServoSubsystem;
import frc.robot.subsystems.drive.AbstractSwerveDrivetrain;
import frc.robot.subsystems.drive.DriveCommands;
import frc.robot.subsystems.scoring.ScoringSuperstructure;
import frc.robot.subsystems.scoring.ScoringSuperstructureAction;
import frc.robot.subsystems.scoring.ScoringSuperstructureState;

public class CommandFactory {
    private static final ScoringSuperstructure scoringSuperstructure = SubsystemManager.getInstance().getScoringSuperstructure();
    private static final AbstractSwerveDrivetrain swerve = SubsystemManager.getInstance().getDrivetrain();
    private static final AbstractClimberSubsystem climber = SubsystemManager.getInstance().getClimberSubsystem();
    private static final ServoSubsystem servo = SubsystemManager.getInstance().getServoSubsystem();

    public static Command autoScoreCoral(Direction direction) {
        return Commands.sequence(
                scoringSuperstructure.setAction(ScoringSuperstructureAction.IDLE),
                swerve.useReefAlignTarget(),
                Commands.deadline(
                        Commands.waitUntil(swerve::shouldStartScoring)
                                .andThen(
                                        scoringSuperstructure.setAction(
                                                switch (Controls.Operator.lastScoringHeight) {
                                                    case 1 -> ScoringSuperstructureAction.SCORE_L1_CORAL;
                                                    case 2 -> ScoringSuperstructureAction.SCORE_L2_CORAL;
                                                    case 3 -> ScoringSuperstructureAction.SCORE_L3_CORAL;
                                                    default -> ScoringSuperstructureAction.SCORE_L4_CORAL;
                                                }
                                        ),
                                        Commands.waitUntil(swerve::atScoringTargetPose),
                                        Commands.waitUntil(() -> scoringSuperstructure.getCurrentState() == ScoringSuperstructureState.EXECUTING_ACTION),
                                        scoringSuperstructure.setUseIntakeSpeed(true),
                                        Commands.waitUntil(() -> scoringSuperstructure.getCurrentState() == ScoringSuperstructureState.DONE),
                                        scoringSuperstructure.setUseIntakeSpeed(false),
                                        Commands.waitUntil(scoringSuperstructure::elevatorMoveThreshold)
                                ),
                        DriveCommands.moveToClosestReefPositionWithPathPlanner(direction, swerve::getPose)
                )
        ).finallyDo(
                () -> {
                    scoringSuperstructure.setSimHasCoral(false);
                    swerve.currentAlignTarget = null;
                }
        );
    }

    public static Command autoDeAlgae() {
        return Commands.sequence(
                scoringSuperstructure.setAction(ScoringSuperstructureAction.IDLE),
                swerve.useAlgaeAlignTarget(),
                Commands.deadline(
                        Commands.waitUntil(swerve::shouldStartScoring)
                                .andThen(
                                        scoringSuperstructure.setAction(
                                                Controls.Driver.lastAlgaeHeight == 2
                                                        ? ScoringSuperstructureAction.INTAKE_L2_ALGAE
                                                        : ScoringSuperstructureAction.INTAKE_L3_ALGAE
                                        ),
                                        Commands.waitUntil(swerve::atScoringTargetPose),
                                        scoringSuperstructure.setUseIntakeSpeed(true),
                                        Commands.waitUntil(() -> !scoringSuperstructure.hasCoral()),
                                        scoringSuperstructure.setAction(ScoringSuperstructureAction.IDLE),
                                        Commands.waitUntil(scoringSuperstructure::elevatorMoveThreshold)
                                ),
                        DriveCommands.moveToClosestReefPositionWithPathPlanner(Direction.ALGAE, swerve::getPose)
                ),
                swerve.useReefAlignTarget()
        );
    }

    public static Command autoCoralIntake() {
        return Commands.deadline(
                Commands.waitUntil(swerve::shouldStartScoring)
                        .andThen(
                                scoringSuperstructure.setAction(ScoringSuperstructureAction.INTAKE_FROM_HP),
                                Commands.waitUntil(swerve::atHPTargetPose),
                                scoringSuperstructure.setSimHasCoralCommand(true)
                        ),
                DriveCommands.moveToClosestHPStation(swerve::getPose)
        ).andThen(
                scoringSuperstructure.setAction(ScoringSuperstructureAction.IDLE)
        );
    }
}
