package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.FieldConstants.TargetPositions;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.util.PoseUtil;

import java.util.List;
import java.util.function.Supplier;
import java.util.stream.Collectors;

public class DriveCommands {
    private static final AbstractSwerveDrivetrain drivetrain = SubsystemManager.getInstance().getDrivetrain();

    public static Command moveToClosestReefPositionWithPID(TargetPositions.Direction direction, Supplier<Pose2d> currentRobotPose) {
        var nearestReefPose = getClosestTarget(currentRobotPose);

        var desiredPose = PoseUtil.ReefRelativeFromDirection(nearestReefPose, direction);

        return drivetrain.directlyMoveTo(desiredPose, true);
    }

    public static Command moveToClosestHPStation(Supplier<Pose2d> currentRobotPose) {
        var nearestHPStation = getClosestHPStation(currentRobotPose);

        return drivetrain.directlyMoveTo(nearestHPStation, false);
    }

    public static Command moveToClosestReefPositionWithPathPlanner(TargetPositions.Direction direction, Supplier<Pose2d> currentRobotPose) {
        var nearestReefPose = getClosestTarget(currentRobotPose);

        var desiredPose = PoseUtil.ReefRelativeFromDirection(nearestReefPose, direction);

        return drivetrain.pathToPoseCommand(desiredPose);
    }

    public static Command moveToClosestReefPositionWithLC(TargetPositions.Direction direction, Supplier<Pose2d> currentRobotPose) {
        var nearestReefPose = getClosestTarget(currentRobotPose);

        var desiredPose = PoseUtil.ReefRelativeFromDirection(nearestReefPose, direction);

        return DriveCommands.drivetrain.fineTuneUsingLaserCANCommand(desiredPose)
                .beforeStarting(
                        () -> drivetrain.setAlignmentDirection(direction)
                )
                .finallyDo(
                        () -> drivetrain.setAlignmentDirection(TargetPositions.Direction.ALGAE)
                );
    }

    public static Command moveToReefPosition(TargetPositions position) {
        var desiredPose = position.getAllianceRespectivePose();
        return DriveCommands.drivetrain.fineTuneUsingLaserCANCommand(desiredPose);
    }

    public static Pose2d getClosestTarget(Supplier<Pose2d> currentRobotPose) {

        List<FieldConstants.TargetPositions> allReefTargets = List.of(
                FieldConstants.TargetPositions.REEF_AB,
                FieldConstants.TargetPositions.REEF_CD,
                FieldConstants.TargetPositions.REEF_EF,
                FieldConstants.TargetPositions.REEF_GH,
                FieldConstants.TargetPositions.REEF_IJ,
                FieldConstants.TargetPositions.REEF_KL
        );

        return currentRobotPose.get().nearest(
                allReefTargets.stream().map(TargetPositions::getAllianceRespectivePose)
                        .collect(Collectors.toList()));
    }

    public static Pose2d getClosestHPStation(Supplier<Pose2d> currentRobotPose) {
        List<TargetPositions> allHPStations = List.of(
                TargetPositions.CORALSTATION_LEFT,
                TargetPositions.CORALSTATION_RIGHT
        );

        return currentRobotPose.get().nearest(
                allHPStations.stream().map(TargetPositions::getAllianceRespectivePose)
                        .collect(Collectors.toList()));
    }
}