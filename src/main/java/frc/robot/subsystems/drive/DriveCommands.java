package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.util.AllianceFlipUtil;
import frc.lib.util.PoseUtil;
import frc.robot.constants.Controls;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.SubsystemManager;

import java.util.List;
import java.util.function.Supplier;
import java.util.stream.Collectors;
import java.util.stream.Stream;

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
                                        PoseUtil.ReefRelativeLeftOf(
                                                targetPosition
                                        )
                                )
                        ),
                        swerve.directlyMoveTo(
                                AllianceFlipUtil.apply(
                                        PoseUtil.ReefRelativeRightOf(
                                                targetPosition
                                        )
                                )
                        ),
                        Controls.Driver.reefLeft
                )
        );
    }

    public static Command pathFindToClosestReefCommand(Supplier<Pose2d> currentRobotPose){
        List<FieldConstants.TargetPositions> allReefTargets = List.of(
                FieldConstants.TargetPositions.REEF_AB,
                FieldConstants.TargetPositions.REEF_CD,
                FieldConstants.TargetPositions.REEF_EF,
                FieldConstants.TargetPositions.REEF_GH,
                FieldConstants.TargetPositions.REEF_IJ,
                FieldConstants.TargetPositions.REEF_KL
        );

        Supplier<Pose2d> nearestReefPose = (() -> currentRobotPose.get().nearest(
                allReefTargets.stream()
                        .map(target -> target.Pose)
                        .collect(Collectors.toList())));
        return swerve.pathfindTo(nearestReefPose.get());
    }
}
