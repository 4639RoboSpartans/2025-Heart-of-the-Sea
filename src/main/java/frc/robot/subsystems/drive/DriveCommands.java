package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.util.AllianceFlipUtil;
import frc.lib.util.PoseUtil;
import frc.robot.constants.Controls;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.SubsystemManager;

public class DriveCommands {
    private static final AbstractSwerveDrivetrain swerve = SubsystemManager.getInstance().getDrivetrain();

    public static Command pathfindToReefCommand(FieldConstants.TargetPositions targetPosition) {
        return Commands.sequence(
                swerve.pathfindTo(
                        AllianceFlipUtil.apply(
                                targetPosition.Pose
                        )
                ),
                Commands.either(
                        swerve.directlyMoveTo(
                                AllianceFlipUtil.apply(
                                        PoseUtil.leftOf(
                                                targetPosition
                                        )
                                )
                        ),
                        swerve.directlyMoveTo(
                                AllianceFlipUtil.apply(
                                        PoseUtil.rightOf(
                                                targetPosition
                                        )
                                )
                        ),
                        Controls.Driver.reefLeft
                )
        );
    }
}
