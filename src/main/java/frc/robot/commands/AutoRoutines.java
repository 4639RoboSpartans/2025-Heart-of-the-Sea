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

    public Auton COMP_J_K() {
        return compileAuton(
            true,
            new ScoringTarget('J', 4),
            new ScoringTarget('K', 4)
        );
    }

    public Auton COMP_H_G() {
        return compileAuton(
            true,
            new ScoringTarget('H', 4),
            new ScoringTarget('G', 4)
        );
    }

    public Auton COMP_A_B() {
        return compileAuton(
            true,
            new ScoringTarget('A', 4),
            new ScoringTarget('B', 4)
        );
    }

    public Auton COMP_H_A() {
        return compileAuton(
            true,
            new ScoringTarget('H', 4),
            new ScoringTarget('A', 4)
        );
    }

    public Auton COMP_G_C_D_B() {
        return compileAuton(
            true,
            new ScoringTarget('G', 4),
            new ScoringTarget('C', 4),
            new ScoringTarget('D', 4),
            new ScoringTarget('B', 4)
        );
    }

    public Auton TEST_A_B() {
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
    private Auton compileAuton(
        boolean isComp,
        ScoringTarget... scoringTargets
        ///List<Integer> scoringHeights, List<Character> scoringLocations
    ) {
        // Create names
        String namePrefix = isComp ? "COMP-" : "TEST-";
        String autonName = getAutonName(scoringTargets, namePrefix);
        String autonPathName = getAutonPathName(scoringTargets, namePrefix);

        // Create path segments
        int numPathSegments = (scoringTargets.length) + (scoringTargets.length - 1);     // Movements to HP station
        AutoRoutine routine = factory.newRoutine(autonName);
        AutoTrajectory[] pathSegments = IntStream.range(0, numPathSegments)
            .mapToObj(i -> routine.trajectory(autonPathName, i))
            .toArray(AutoTrajectory[]::new);

        // Create commands
        List<Command> commands = new ArrayList<>();
        commands.add(pathSegments[0].resetOdometry());
        for (int targetIndex = 0; targetIndex < scoringTargets.length; targetIndex++) {
            addScoringSegment(commands, pathSegments[targetIndex * 2], scoringTargets[targetIndex]);
            if (targetIndex != scoringTargets.length - 1) {
                addHPLoadingSegment(commands, pathSegments[targetIndex * 2 + 1]);
            }
        }

        // Activate the commands when the auton routine is active
        routine.active().onTrue(CommandsUtil.sequence(commands));

        return new Auton(routine, autonName);
    }

    private void addScoringSegment(List<Command> commands, AutoTrajectory path, ScoringTarget scoringTarget) {
        // Add path to scoring
        commands.add(path.cmd());

        // Add directly move to and fine tune stuff IDK
        TargetPositions targetPosition = scoringTarget.getTargetPosition();
        addDirectlyMoveToCommand(
            commands, targetPosition.getPose()
        );
        commands.add(targetPosition.fineTuneTargetCommand.get());

        // Add scoring command
        commands.add(switch (scoringTarget.scoringHeight()) {
            case 1 -> AutoCommands.L1Score.get();
            case 2 -> AutoCommands.L2Score.get();
            case 3 -> AutoCommands.L3Score.get();
            case 4 -> AutoCommands.L4Score.get();
            default -> AutoCommands.HPLoad.get().withTimeout(1);
        });
    }

    private void addHPLoadingSegment(List<Command> commands, AutoTrajectory path) {
        // Add path to intake
        commands.add(path.cmd());

        // Add directly move to stuff IDK
        addDirectlyMoveToCommand(
            commands, TargetPositions.CORALSTATION_LEFT.getPose()
        );

        // Add HP load command
        commands.add(AutoCommands.HPLoad.get().withTimeout(1));
    }

    private static void addDirectlyMoveToCommand(List<Command> commands, Pose2d targetPose) {
        AbstractSwerveDrivetrain drivetrain = SubsystemManager.getInstance().getDrivetrain();

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

    private static String getAutonPathName(ScoringTarget[] scoringTargets, String namePrefix) {
        return namePrefix + Arrays.stream(scoringTargets)
            .map(i -> String.valueOf(i.scoringLocation))
            .collect(Collectors.joining("-"));
    }

    private static String getAutonName(ScoringTarget[] scoringTargets, String namePrefix) {
        return namePrefix + Arrays.stream(scoringTargets)
            .map(ScoringTarget::toString)
            .collect(Collectors.joining("-"));
    }

    public List<Auton> getAllCompRoutines() {
        return List.of(
            COMP_A_B(),
            COMP_H_A(),
            COMP_H_G(),
            COMP_J_K(),
            COMP_G_C_D_B()
        );
    }

    public record Auton(AutoRoutine routine, String name){}
}