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
import frc.robot.subsystems.vision.Limelights;

import java.lang.annotation.Target;
import java.util.Comparator;
import java.util.List;
import java.util.function.BinaryOperator;
import java.util.function.Supplier;
import java.util.stream.Collectors;
import java.util.stream.Stream;

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
                                )
                        ),
                        drivetrain.directlyMoveTo(
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

    public static Command moveToNearestReefCenterPosition() {
        Supplier<Pose2d> currentRobotPose = drivetrain::getPose;
        List<FieldConstants.TargetPositions> allReefTargets = List.of(
                FieldConstants.TargetPositions.REEF_AB,
                FieldConstants.TargetPositions.REEF_CD,
                FieldConstants.TargetPositions.REEF_EF,
                FieldConstants.TargetPositions.REEF_GH,
                FieldConstants.TargetPositions.REEF_IJ,
                FieldConstants.TargetPositions.REEF_KL
        );

        FieldConstants.TargetPositions target = allReefTargets.stream().reduce(FieldConstants.TargetPositions.REEF_AB, (target1, target2) -> {
            if (currentRobotPose.get().nearest(List.of(target1.getPose(), target1.getPose()))==target1.getPose()) return target1;
            else return target2;
        });

        var nearestReefPose = target.getPose();
        var visionPoseSupplier = Limelights.RIGHT.createVisionAlignPoseSupplier(target.getAprilTagIDHolder());
        return Commands.runOnce(() -> Limelights.RIGHT.filterRawFiducials(target.getAprilTagIDHolder().getAllianceRespectiveID()))
                .andThen( DriveCommands.drivetrain.directlyMoveTo(nearestReefPose, visionPoseSupplier)
                        .until(new Trigger(() -> PoseUtil.withinTolerance(nearestReefPose, visionPoseSupplier.get(), Units.inchesToMeters(0.5))).debounce(0.1)))
                .finallyDo(Limelights.RIGHT::resetFiducialFilter);
    }

    public static Command moveToNearestReefLeftPosition() {
        Supplier<Pose2d> currentRobotPose = drivetrain::getPose;
        List<FieldConstants.TargetPositions> allReefTargets = List.of(
                FieldConstants.TargetPositions.REEF_AB,
                FieldConstants.TargetPositions.REEF_CD,
                FieldConstants.TargetPositions.REEF_EF,
                FieldConstants.TargetPositions.REEF_GH,
                FieldConstants.TargetPositions.REEF_IJ,
                FieldConstants.TargetPositions.REEF_KL
        );

        FieldConstants.TargetPositions target = allReefTargets.stream().reduce(FieldConstants.TargetPositions.REEF_AB, (target1, target2) -> {
            if (currentRobotPose.get().nearest(List.of(target1.getPose(), target1.getPose()))==target1.getPose()) return target1;
            else return target2;
        });

        var nearestReefPose = PoseUtil.ReefRelativeLeftOf(target.getPose());
        var visionPoseSupplier = Limelights.RIGHT.createVisionAlignPoseSupplier(target.getAprilTagIDHolder());
        return Commands.runOnce(() -> Limelights.RIGHT.filterRawFiducials(target.getAprilTagIDHolder().getAllianceRespectiveID()))
                .andThen( DriveCommands.drivetrain.directlyMoveTo(nearestReefPose, visionPoseSupplier)
                        .until(new Trigger(() -> PoseUtil.withinTolerance(nearestReefPose, visionPoseSupplier.get(), Units.inchesToMeters(0.5))).debounce(0.1)))
                .finallyDo(Limelights.RIGHT::resetFiducialFilter);
    }

    public static Command moveToNearestReefRightPosition() {
        Supplier<Pose2d> currentRobotPose = drivetrain::getPose;
        List<FieldConstants.TargetPositions> allReefTargets = List.of(
                FieldConstants.TargetPositions.REEF_AB,
                FieldConstants.TargetPositions.REEF_CD,
                FieldConstants.TargetPositions.REEF_EF,
                FieldConstants.TargetPositions.REEF_GH,
                FieldConstants.TargetPositions.REEF_IJ,
                FieldConstants.TargetPositions.REEF_KL
        );

        FieldConstants.TargetPositions target = allReefTargets.stream().reduce(FieldConstants.TargetPositions.REEF_AB, (target1, target2) -> {
            if (currentRobotPose.get().nearest(List.of(target1.getPose(), target1.getPose()))==target1.getPose()) return target1;
            else return target2;
        });

        var nearestReefPose = PoseUtil.ReefRelativeRightOf(target.getPose());
        var visionPoseSupplier = Limelights.LEFT.createVisionAlignPoseSupplier(target.getAprilTagIDHolder());
        return Commands.runOnce(() -> Limelights.LEFT.filterRawFiducials(target.getAprilTagIDHolder().getAllianceRespectiveID()))
                .andThen( DriveCommands.drivetrain.directlyMoveTo(nearestReefPose, visionPoseSupplier)
                .until(new Trigger(() -> PoseUtil.withinTolerance(nearestReefPose, visionPoseSupplier.get(), Units.inchesToMeters(0.5))).debounce(0.1)))
                .finallyDo(Limelights.LEFT::resetFiducialFilter);
    }

    public static Command moveToDesiredCoralStationPosition(boolean left) {
        Supplier<Pose2d> currentRobotPose = drivetrain::getPose;
        var desiredTarget = left ? FieldConstants.TargetPositions.CORALSTATION_LEFT : FieldConstants.TargetPositions.CORALSTATION_RIGHT;

        return (!PoseUtil.withinTolerance(desiredTarget.getPose(), currentRobotPose.get(), 2)
                ? drivetrain.pathfindTo(desiredTarget.getPose())
                : drivetrain.directlyMoveTo(desiredTarget.getPose()))
                .until(() -> PoseUtil.withinTolerance(desiredTarget.getPose(), currentRobotPose.get(), 2))
                .andThen(drivetrain.directlyMoveTo(desiredTarget.getPose()));
    }

    public static Command moveToProcessor(){
        var desiredTarget = FieldConstants.TargetPositions.PROCESSOR;
        Supplier<Pose2d> currentRobotPose = SubsystemManager.getInstance().getDrivetrain()::getPose;
        return (!PoseUtil.withinTolerance(desiredTarget.getPose(), currentRobotPose.get(), 2)
                        ? drivetrain.pathfindTo(desiredTarget.getPose())
                        : drivetrain.directlyMoveTo(desiredTarget.getPose()))
                .until(() -> PoseUtil.withinTolerance(desiredTarget.getPose(), currentRobotPose.get(), 2))
                .andThen(drivetrain.directlyMoveTo(desiredTarget.getPose()));
    }
}