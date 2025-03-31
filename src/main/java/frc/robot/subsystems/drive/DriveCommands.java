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

        return DriveCommands.drivetrain.directlyMoveTo(desiredPose, true);
    }

    public static Command moveToClosestHPStation(Supplier<Pose2d> currentRobotPose) {
        var nearestHPStation = getClosestHPStation(currentRobotPose);

        return DriveCommands.drivetrain.directlyMoveTo(nearestHPStation, false);
    }

    public static Command moveToClosestReefPositionWithPathPlanner(TargetPositions.Direction direction, Supplier<Pose2d> currentRobotPose) {
        var nearestReefPose = getClosestTarget(currentRobotPose);

        var desiredPose = PoseUtil.ReefRelativeFromDirection(nearestReefPose, direction);

        return DriveCommands.drivetrain.pathToPoseCommand(desiredPose)
                .until(new Trigger(() -> PoseUtil.withinTolerance(desiredPose, currentRobotPose.get(), Units.inchesToMeters(2))).debounce(0.1));
    }

    public static Command moveToClosestReefPositionWithLC(TargetPositions.Direction direction, Supplier<Pose2d> currentRobotPose) {
        var nearestReefPose = getClosestTarget(currentRobotPose);

        var desiredPose = PoseUtil.ReefRelativeFromDirection(nearestReefPose, direction);

        return DriveCommands.drivetrain.fineTuneUsingLaserCANCommand(desiredPose)
                .until(new Trigger(() -> PoseUtil.withinTolerance(desiredPose, currentRobotPose.get(), Units.inchesToMeters(2))).debounce(0.1));
    }

    public static Command moveToReefPosition(TargetPositions position) {
        var desiredPose = position.getAllianceRespectivePose();
        return DriveCommands.drivetrain.fineTuneUsingLaserCANCommand(desiredPose);
    }

    public static Pose2d getClosestTarget(Supplier<Pose2d> currentRobotPose) {
        List<TargetPositions> allReefTargets = List.of(
            TargetPositions.REEF_AB,
            TargetPositions.REEF_CD,
            TargetPositions.REEF_EF,
            TargetPositions.REEF_GH,
            TargetPositions.REEF_IJ,
            TargetPositions.REEF_KL
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