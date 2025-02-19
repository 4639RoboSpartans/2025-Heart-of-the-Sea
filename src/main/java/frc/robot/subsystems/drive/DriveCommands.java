package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.FunctionalTrigger;
import frc.lib.util.AllianceFlipUtil;
import frc.lib.util.PoseUtil;
import frc.robot.constants.Controls;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.SubsystemManager;

import java.util.List;
import java.util.function.Supplier;
import java.util.stream.Collectors;

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
                        Controls.Driver.targetLeft
                )
        );
    }

    public static Command moveToClosestReefPosition(Supplier<Pose2d> currentRobotPose){
        List<FieldConstants.TargetPositions> allReefTargets = List.of(
                FieldConstants.TargetPositions.REEF_AB,
                FieldConstants.TargetPositions.REEF_CD,
                FieldConstants.TargetPositions.REEF_EF,
                FieldConstants.TargetPositions.REEF_GH,
                FieldConstants.TargetPositions.REEF_IJ,
                FieldConstants.TargetPositions.REEF_KL
        );

        Pose2d nearestReefPose = currentRobotPose.get().nearest(
                allReefTargets.stream()
                        .map(target -> target.Pose)
                        .collect(Collectors.toList()));

        var desiredPose =
                Controls.Driver.targetLeft.getAsBoolean()
                        ? PoseUtil.ReefRelativeLeftOf(nearestReefPose)
                        : Controls.Driver.targetRight.getAsBoolean()
                            ? PoseUtil.ReefRelativeRightOf(nearestReefPose)
                            : nearestReefPose;

        return swerve.directlyMoveTo(desiredPose).until(FunctionalTrigger.of(() -> PoseUtil.withinTolerance(desiredPose, currentRobotPose.get(), Units.inchesToMeters(1))).debounce(0.1));
    }

    public static Command moveToDesiredCoralStationPosition(Supplier<Pose2d> currentRobotPose){
        var desiredPose = Controls.Driver.targetLeft.getAsBoolean()
                ? FieldConstants.TargetPositions.CORALSTATION_LEFT
                : Controls.Driver.targetRight.getAsBoolean()
                    ? FieldConstants.TargetPositions.CORALSTATION_RIGHT
                    : null;

        return desiredPose != null
                ? swerve.pathfindTo(desiredPose.Pose).until(FunctionalTrigger.of(() -> PoseUtil.withinTolerance(desiredPose.Pose, currentRobotPose.get(), Units.inchesToMeters(1))).debounce(0.1))
                : Commands.none();
    }
}
