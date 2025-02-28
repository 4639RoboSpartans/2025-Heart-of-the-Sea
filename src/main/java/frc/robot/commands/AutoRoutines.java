package frc.robot.commands;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.CommandsUtil;
import frc.lib.util.PoseUtil;
import frc.robot.constants.FieldConstants.TargetPositions;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.drive.AbstractSwerveDrivetrain;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.IntStream;

public class AutoRoutines {
    private final AutoFactory factory;

    public AutoRoutines(AutoFactory factory) {
        this.factory = factory;
    }

    public AutoRoutine COMP_J_K() {
        return compileAuton(
            true,
            new ScoringTarget('J', 4),
            new ScoringTarget('K', 4)
        );
    }

    public AutoRoutine COMP_H_G() {
        return compileAuton(
            true,
            new ScoringTarget('H', 4),
            new ScoringTarget('G', 4)
        );
    }

    public AutoRoutine COMP_A_B() {
        return compileAuton(
            true,
            new ScoringTarget('A', 4),
            new ScoringTarget('B', 4)
        );
    }

    public AutoRoutine COMP_H_A() {
        return compileAuton(
            true,
            new ScoringTarget('H', 4),
            new ScoringTarget('A', 4)
        );
    }

    public AutoRoutine COMP_G_C_D_B() {
        return compileAuton(
            true,
            new ScoringTarget('G', 4),
            new ScoringTarget('C', 4),
            new ScoringTarget('D', 4),
            new ScoringTarget('B', 4)
        );
    }

    public AutoRoutine TEST_A_B() {
        return compileAuton(
            false,
            new ScoringTarget('A', 1),
            new ScoringTarget('B', 2)
        );
    }

    private record ScoringTarget(char scoringLocation, int scoringHeight) {
        @Override
        public String toString() {
            return "%s%d".formatted(scoringLocation, scoringHeight);
        }

        public TargetPositions getTargetPosition() {
            return TargetPositions.valueOf(
                "REEF_" + this.scoringLocation
            );
        }
    }

    /**
     * @param isComp         is the auton for competition
     * @param scoringTargets all scoring targets
     *
     * @return a new routine with the specified characteristics
     */
    private AutoRoutine compileAuton(
        boolean isComp,
        ScoringTarget... scoringTargets
        ///List<Integer> scoringHeights, List<Character> scoringLocations
    ) {
        String pathPrefix = isComp ? "COMP-" : "TEST-";
        String name = getAutonName(scoringTargets, pathPrefix);
        String pathName = getAutonPathName(scoringTargets, pathPrefix);

        int numScores = scoringTargets.length;
        int numPaths = (numScores) // Movements to reef
            + (numScores - 1);     // Movements to HP station

        AutoRoutine routine = factory.newRoutine(name);
        List<AutoTrajectory> paths = IntStream.range(0, numPaths)
            .mapToObj(i -> routine.trajectory(pathName, i))
            .toList();

        List<Command> commands = new ArrayList<>();
        commands.add(paths.get(0).resetOdometry());

        for (int pathIndex = 0; pathIndex < numPaths; pathIndex++) {
            commands.add(paths.get(pathIndex).cmd());
            Pose2d targetPose;
            AbstractSwerveDrivetrain drivetrain = SubsystemManager.getInstance().getDrivetrain();
            if (pathIndex % 2 == 0) {
                targetPose = (
                    scoringTargets[pathIndex / 2].getTargetPosition().getPose()
                );
                commands.add(
                    drivetrain
                        .directlyMoveTo(targetPose)
                        .until(
                            () -> PoseUtil.withinTolerance(
                                targetPose,
                                drivetrain.getPose(),
                                Units.inchesToMeters(0.5))
                        )
                        .andThen(scoringTargets[pathIndex / 2].getTargetPosition().fineTuneTargetCommand.get()));
            } else {
                targetPose = (
                    TargetPositions.CORALSTATION_LEFT.getPose()
                );
                commands.add(
                    drivetrain
                        .directlyMoveTo(targetPose)
                        .until(
                            () -> PoseUtil.withinTolerance(
                                targetPose,
                                drivetrain.getPose(),
                                Units.inchesToMeters(0.5))
                        )
                );
            }


            commands.add(
                pathIndex % 2 == 0 ?
                    switch (scoringTargets[pathIndex / 2].scoringHeight()) {
                        case 1 -> AutoCommands.L1Score.get();
                        case 2 -> AutoCommands.L2Score.get();
                        case 3 -> AutoCommands.L3Score.get();
                        case 4 -> AutoCommands.L4Score.get();
                        default -> AutoCommands.HPLoad.get().withTimeout(1);
                    } : AutoCommands.HPLoad.get().withTimeout(1)
            );
        }

        routine.active().onTrue(
            CommandsUtil.sequence(commands)
        );
        return routine;
    }

    private static String getAutonPathName(ScoringTarget[] scoringTargets, String pathPrefix) {
        return pathPrefix + Arrays.stream(scoringTargets)
            .map(i -> String.valueOf(i.scoringLocation))
            .collect(Collectors.joining("-"));
    }

    private static String getAutonName(ScoringTarget[] scoringTargets, String pathPrefix) {
        return pathPrefix + Arrays.stream(scoringTargets)
            .map(ScoringTarget::toString)
            .collect(Collectors.joining("-"));
    }

    public List<AutoRoutine> getAllCompRoutines() {
        return List.of(
            COMP_A_B(),
            COMP_H_A(),
            COMP_H_G(),
            COMP_J_K(),
            COMP_G_C_D_B()
        );
    }

}