package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
                                targetPosition.getPose()
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
                        Controls.Driver.alignReefLeft
                )
        );
    }

    public static Command moveToClosestReefPositionWithTransformation(byte direction) {
        Supplier<Pose2d> currentRobotPose = SubsystemManager.getInstance().getDrivetrain()::getPose;
        List<FieldConstants.TargetPositions> allReefTargets = List.of(
                FieldConstants.TargetPositions.REEF_AB,
                FieldConstants.TargetPositions.REEF_CD,
                FieldConstants.TargetPositions.REEF_EF,
                FieldConstants.TargetPositions.REEF_GH,
                FieldConstants.TargetPositions.REEF_IJ,
                FieldConstants.TargetPositions.REEF_KL
        );

        Pose2d nearestReefPose = currentRobotPose.get().nearest(
                Stream.concat(
                        allReefTargets.stream().map(FieldConstants.TargetPositions::getPose),
                        allReefTargets.stream().map(FieldConstants.TargetPositions::getOpponentAlliancePose)
                ).collect(Collectors.toList()));

        var desiredPose =
                (direction == 0
                        ? PoseUtil.ReefRelativeLeftOf(nearestReefPose)
                        : (direction == 1
                        ? PoseUtil.ReefRelativeRightOf(nearestReefPose)
                        : nearestReefPose));

        return swerve.directlyMoveTo(desiredPose)
                .until(new Trigger(() -> PoseUtil.withinTolerance(desiredPose, currentRobotPose.get(), Units.inchesToMeters(2))).debounce(0.1));
    }

    public static Command moveToClosestReefPositionHardcoded(byte direction) {
        Supplier<Pose2d> currentRobotPose = SubsystemManager.getInstance().getDrivetrain()::getPose;
        List<FieldConstants.TargetPositions> allReefTargets = new java.util.ArrayList<>(List.of(
                FieldConstants.TargetPositions.REEF_AB,
                FieldConstants.TargetPositions.REEF_CD,
                FieldConstants.TargetPositions.REEF_EF,
                FieldConstants.TargetPositions.REEF_GH,
                FieldConstants.TargetPositions.REEF_IJ,
                FieldConstants.TargetPositions.REEF_KL
        ));

        allReefTargets.sort((a, b) -> {
            double diff = Math.hypot(
                    swerve.getPose().getX() - a.getPose().getX(),
                    swerve.getPose().getY() - a.getPose().getY()
            ) - Math.hypot(
                    swerve.getPose().getX() - b.getPose().getX(),
                    swerve.getPose().getY() - b.getPose().getY()
            );
            return diff == 0 ? 0 : diff < 0 ? -1 : 1;
        });

        var desiredPose = switch (direction) {
            case 0 -> allReefTargets.get(0).leftPose;
            case 1 -> allReefTargets.get(0).rightPose;
            default -> allReefTargets.get(0).getPose();
        };

        return swerve.directlyMoveTo(desiredPose)
                .until(
                        new Trigger(
                                () -> PoseUtil.withinTolerance(
                                        desiredPose,
                                        currentRobotPose.get(),
                                        Units.inchesToMeters(2)
                                )
                        )
                                .debounce(0.1));
    }

    public static Command moveToDesiredCoralStationPosition(boolean left) {
        Supplier<Pose2d> currentRobotPose = SubsystemManager.getInstance().getDrivetrain()::getPose;
        var desiredTarget = left ? FieldConstants.TargetPositions.CORALSTATION_LEFT : FieldConstants.TargetPositions.CORALSTATION_RIGHT;

        return (!PoseUtil.withinTolerance(desiredTarget.getPose(), currentRobotPose.get(), 2)
                ? swerve.pathfindTo(desiredTarget.getPose())
                : swerve.directlyMoveTo(desiredTarget.getPose()))
                .until(() -> PoseUtil.withinTolerance(desiredTarget.getPose(), currentRobotPose.get(), 2))
                .andThen(swerve.directlyMoveTo(desiredTarget.getPose()));
    }

    public static Command moveToProcessor(){
        var desiredTarget = FieldConstants.TargetPositions.PROCESSOR;
        Supplier<Pose2d> currentRobotPose = SubsystemManager.getInstance().getDrivetrain()::getPose;
        return (!PoseUtil.withinTolerance(desiredTarget.getPose(), currentRobotPose.get(), 2)
                        ? swerve.pathfindTo(desiredTarget.getPose())
                        : swerve.directlyMoveTo(desiredTarget.getPose()))
                .until(() -> PoseUtil.withinTolerance(desiredTarget.getPose(), currentRobotPose.get(), 2))
                .andThen(swerve.directlyMoveTo(desiredTarget.getPose()));
    }
}