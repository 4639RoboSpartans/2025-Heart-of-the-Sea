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
import frc.robot.constants.FieldConstants.TargetPositions;
import frc.robot.subsystems.SubsystemManager;

import java.util.List;
import java.util.function.Supplier;
import java.util.stream.Collectors;

public class DriveCommands {
    private static final AbstractSwerveDrivetrain drivetrain = SubsystemManager.getInstance().getDrivetrain();

    public static Command pathfindToReefCommand(FieldConstants.TargetPositions targetPosition) {
        return Commands.sequence(
            drivetrain.pathfindTo(
                AllianceFlipUtil.apply(
                    targetPosition.getPose()
                )
            ),
            Commands.either(
                drivetrain.directlyMoveTo(
                    AllianceFlipUtil.apply(
                        PoseUtil.ReefRelativeLeftOf(
                            targetPosition
                        )
                    ),
                    drivetrain::getPose
                ),
                drivetrain.directlyMoveTo(
                    AllianceFlipUtil.apply(
                        PoseUtil.ReefRelativeRightOf(
                            targetPosition
                        )
                    ),
                    drivetrain::getPose
                ),
                Controls.Driver.alignReefLeft
            )
        );
    }

    public static Command moveToClosestReefPositionWithTransformation(byte direction, Supplier<Pose2d> currentRobotPose) {
        drivetrain.setVisionStandardDeviations(0.1, 0.1, 10);
        List<FieldConstants.TargetPositions> allReefTargets = List.of(
            FieldConstants.TargetPositions.REEF_AB,
            FieldConstants.TargetPositions.REEF_CD,
            FieldConstants.TargetPositions.REEF_EF,
            FieldConstants.TargetPositions.REEF_GH,
            FieldConstants.TargetPositions.REEF_IJ,
            FieldConstants.TargetPositions.REEF_KL
        );

        // Pose2d nearestReefPose = currentRobotPose.get().nearest(
        //         Stream.concat(
        //                 allReefTargets.stream().map(FieldConstants.TargetPositions::getPose),
        //                 allReefTargets.stream().map(FieldConstants.TargetPositions::getOpponentAlliancePose)
        //         ).collect(Collectors.toList()));

        Pose2d nearestReefPose = currentRobotPose.get().nearest(
            allReefTargets.stream().map(FieldConstants.TargetPositions::getAllianceRespectivePose)
                .collect(Collectors.toList()));

        var desiredPose =
            (direction == 0
                ? PoseUtil.ReefRelativeLeftOf(nearestReefPose)
                : (direction == 1
                ? PoseUtil.ReefRelativeRightOf(nearestReefPose)
                : nearestReefPose));

        return DriveCommands.drivetrain.directlyMoveTo(desiredPose, currentRobotPose)
            .until(new Trigger(() -> PoseUtil.withinTolerance(desiredPose, currentRobotPose.get(), Units.inchesToMeters(2))).debounce(0.1));
    }

    public static Command moveToReefPosition(TargetPositions position, Supplier<Pose2d> currentRobotPose) {
        var desiredPose = position.getAllianceRespectivePose();

        return DriveCommands.drivetrain.directlyMoveTo(desiredPose, currentRobotPose)
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
                drivetrain.getPose().getX() - a.getPose().getX(),
                drivetrain.getPose().getY() - a.getPose().getY()
            ) - Math.hypot(
                drivetrain.getPose().getX() - b.getPose().getX(),
                drivetrain.getPose().getY() - b.getPose().getY()
            );
            return diff == 0 ? 0 : diff < 0 ? -1 : 1;
        });

        var desiredPose = switch (direction) {
            case 0 -> allReefTargets.get(0).leftPose;
            case 1 -> allReefTargets.get(0).rightPose;
            default -> allReefTargets.get(0).getPose();
        };

        return drivetrain.directlyMoveTo(desiredPose, currentRobotPose)
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
            ? drivetrain.pathfindTo(desiredTarget.getPose())
            : drivetrain.directlyMoveTo(desiredTarget.getPose(), currentRobotPose))
            .until(() -> PoseUtil.withinTolerance(desiredTarget.getPose(), currentRobotPose.get(), 2))
            .andThen(drivetrain.directlyMoveTo(desiredTarget.getPose(), currentRobotPose));
    }

    public static Command moveToProcessor() {
        var desiredTarget = FieldConstants.TargetPositions.PROCESSOR;
        Supplier<Pose2d> currentRobotPose = SubsystemManager.getInstance().getDrivetrain()::getPose;
        return (!PoseUtil.withinTolerance(desiredTarget.getPose(), currentRobotPose.get(), 2)
            ? drivetrain.pathfindTo(desiredTarget.getPose())
            : drivetrain.directlyMoveTo(desiredTarget.getPose(), currentRobotPose))
            .until(() -> PoseUtil.withinTolerance(desiredTarget.getPose(), currentRobotPose.get(), 2))
            .andThen(drivetrain.directlyMoveTo(desiredTarget.getPose(), currentRobotPose));
    }

    /**
     * only for use in auton
     *
     * @param reefLetter letter
     *
     * @return command
     */
    public static Command moveToHexThenMoveToRLCommand(char reefLetter) {
        return FieldConstants.TargetPositions.hexReefPoseFromChar(reefLetter)
            .map(target -> (Command) drivetrain.directlyMoveTo(target.getAllianceRespectivePose(), drivetrain::getPose).until(() -> PoseUtil.withinTolerance(drivetrain.getPose(), target.getAllianceRespectivePose(), Units.inchesToMeters(2))))
            .orElse(Commands.none())
            .andThen(drivetrain.directlyMoveTo(FieldConstants.TargetPositions.RLReefPoseFromChar(reefLetter).getAllianceRespectivePose(), drivetrain::getPose))
            .until(() -> PoseUtil.withinTolerance(drivetrain.getPose(), FieldConstants.TargetPositions.RLReefPoseFromChar(reefLetter).getAllianceRespectivePose(), Units.inchesToMeters(0.5)));

    }
}