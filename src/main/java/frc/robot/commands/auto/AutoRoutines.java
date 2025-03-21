package frc.robot.commands.auto;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.util.CommandsUtil;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.FieldConstants.TargetPositions;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.drive.DriveCommands;
import frc.robot.subsystems.scoring.ScoringSuperstructure;
import frc.robot.subsystems.scoring.ScoringSuperstructureState;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.Supplier;
import java.util.stream.Collectors;
import java.util.stream.IntStream;

public class AutoRoutines {
    private final Supplier<AutoFactory> factory;

    private final ScoringSuperstructure scoringSuperstructure = SubsystemManager.getInstance().getScoringSuperstructure();

    public AutoRoutines(Supplier<AutoFactory> factory) {
        this.factory = factory;
    }

    public AutonSupplier NONE() {
        return new AutonSupplier(
                () -> factory.get().newRoutine("NONE"),
                "NONE"
        );
    }

    public AutonSupplier COMP_J_K() {
        return new AutonSupplier(
            () -> compileAuton(
                true,
                true,
                new ScoringTarget('J', 4),
                new ScoringTarget('K', 4)
            ),
            getAutonName(
                new ScoringTarget[] {
                    new ScoringTarget('J', 4),
                    new ScoringTarget('K', 4)
                }, 
                "COMP-"
            )
        );
    }

    public AutonSupplier COMP_J_L() {
        return new AutonSupplier(
            () -> compileAuton(
                true, 
                true, 
                new ScoringTarget('J', 4),
                new ScoringTarget('L', 4)
            ),
            getAutonName(
                new ScoringTarget[] {
                    new ScoringTarget('J', 4),
                    new ScoringTarget('L', 4)
                }, 
                "COMP-"
            )
        );
    }

    public AutonSupplier COMP_H_G() {
        return new AutonSupplier(
            () -> compileAuton(
                true, 
                false, 
                new ScoringTarget('H', 4),
                new ScoringTarget('G', 4)
            ),
            getAutonName(
                new ScoringTarget[] {
                    new ScoringTarget('H', 4),
                    new ScoringTarget('G', 4)
                }, 
                "COMP-"
            )
        );
    }

    public AutonSupplier COMP_A_B() {
        return new AutonSupplier(
            () -> compileAuton(
                true, 
                true, 
                new ScoringTarget('A', 4),
                new ScoringTarget('B', 4)
            ),
            getAutonName(
                new ScoringTarget[] {
                    new ScoringTarget('A', 4),
                    new ScoringTarget('B', 4)
                }, 
                "COMP-"
            )
        );
    }

    public AutonSupplier COMP_H_A() {
        return new AutonSupplier(
            () -> compileAuton(
                true, 
                true, 
                new ScoringTarget('H', 4),
                new ScoringTarget('A', 4)
            ),
            getAutonName(
                new ScoringTarget[] {
                    new ScoringTarget('H', 4),
                    new ScoringTarget('A', 4)
                }, 
                "COMP-"
            )
        );
    }

    public AutonSupplier COMP_G_C_D_B() {
        return new AutonSupplier(
            () -> compileAuton(
            true,
            false,
            new ScoringTarget('G', 4),
            new ScoringTarget('C', 4),
            new ScoringTarget('D', 4),
            new ScoringTarget('B', 4)
            ),
            getAutonName(
                new ScoringTarget[] {
                    new ScoringTarget('G', 4),
                    new ScoringTarget('C', 4),
                    new ScoringTarget('D', 4),
                    new ScoringTarget('B', 4)
                }, 
                "COMP-"
            )
        );
    }

    public AutonSupplier COMP_I_K_L() {
        return new AutonSupplier(
            () -> compileAuton(
            true,
            true,
            new ScoringTarget('I', 4),
            new ScoringTarget('K', 4),
            new ScoringTarget('L', 4)
            ),
            getAutonName(
                new ScoringTarget[] {
                    new ScoringTarget('I', 4),
                    new ScoringTarget('K', 4),
                    new ScoringTarget('L', 4)
                }, 
                "COMP-"
            )
        );
    }

    
    public AutonSupplier TEST_E_C() {
        return new AutonSupplier(
            () -> compileAuton(
                false, 
                false, 
                new ScoringTarget('E', 4),
                new ScoringTarget('C', 4)
            ),
            getAutonName(
                new ScoringTarget[] {
                    new ScoringTarget('E', 4),
                    new ScoringTarget('C', 4)
                }, 
                "COMP-"
            )
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
        boolean left,
        ScoringTarget... scoringTargets
        ///List<Integer> scoringHeights, List<Character> scoringLocations
    ) {
        // Create names
        String namePrefix = isComp ? "COMP-" : "TEST-";
        String autonName = getAutonName(scoringTargets, namePrefix);
        String autonPathName = getAutonPathName(scoringTargets, namePrefix);

        // Create path segments
        int numPathSegments = (scoringTargets.length) + (scoringTargets.length - 1);     // Movements to HP station
        AutoRoutine routine = factory.get().newRoutine(autonName);
        AutoTrajectory[] pathSegments = IntStream.range(0, numPathSegments)
            .mapToObj(i -> routine.trajectory(autonPathName, i))
            .toArray(AutoTrajectory[]::new);

        // Create commands
        List<Command> commands = new ArrayList<>();
        commands.add(pathSegments[0].resetOdometry());
        for (int targetIndex = 0; targetIndex < scoringTargets.length; targetIndex++) {
            addScoringSegment(commands, pathSegments[targetIndex * 2], scoringTargets[targetIndex]);
            if (targetIndex != scoringTargets.length - 1) {
                addHPLoadingSegment(commands, pathSegments[targetIndex * 2 + 1], left);
            }
        }

        // Activate the commands when the auton routine is active
        routine.active().onTrue(
                CommandsUtil.sequence(commands)
        );

        return routine;
    }

    private void addScoringSegment(List<Command> commands, AutoTrajectory path, ScoringTarget scoringTarget) {
        // Add the path to scoring
        commands.add(path.cmd());
        path.atTime("L4_UP").onTrue(
                switch (scoringTarget.scoringHeight()) {
                    case 1 -> AutoCommands.L1Score.get();
                    case 2 -> AutoCommands.L2Score.get();
                    case 3 -> AutoCommands.L3Score.get();
                    case 4 -> AutoCommands.L4Score.get();
                    default -> AutoCommands.HPLoad.get().withTimeout(1);
                }
        );

        if (AutoConstants.addVisionAlignToCommands) {
            TargetPositions targetPosition = scoringTarget.getTargetPosition();
            addDirectlyMoveToCommand(
                commands, targetPosition
            );
        }

        commands.add(
            Commands.deadline(
                Commands.waitUntil(() -> scoringSuperstructure.getCurrentState() == ScoringSuperstructureState.EXECUTING_ACTION)
                    .andThen(Commands.waitUntil(() -> !scoringSuperstructure.hasCoral())),
                AutoCommands.SwerveStop.get()
            )
        );
    }

    @SuppressWarnings("unused")
    private void addHPLoadingSegment(List<Command> commands, AutoTrajectory path, boolean isStationLeft) {
        commands.add(Commands.runOnce(() -> SubsystemManager.getInstance().getDrivetrain().setVisionStandardDeviations(100, 100, 100)));
        // Add path to intake
        commands.add(path.cmd());
        commands.add(Commands.runOnce(() -> SubsystemManager.getInstance().getDrivetrain().setVisionStandardDeviations(0.1, 0.1, 1)));

        // Add directly move to stuff IDK
        if (AutoConstants.addVisionAlignToCommands && AutoConstants.addHPVisionAlignToCommands) {
            addHPMoveToCommand(
                commands,
                isStationLeft
            );
        }

        // Add HP load command
        commands.add(AutoCommands.HPLoad.get());
    }

    private static void addDirectlyMoveToCommand(List<Command> commands, FieldConstants.TargetPositions targetPose) {
        commands.add(
                DriveCommands.moveToReefPosition(
                    targetPose,
                    () -> {
                        return SubsystemManager.getInstance().getDrivetrain().getPose();
                    }
                )
//                DriveCommands.moveToHexThenMoveToRLCommand(targetPose.toString().charAt(targetPose.toString().length() - 1))
        );
    }

    private static void addHPMoveToCommand(List<Command> commands, boolean isLeft){
        addDirectlyMoveToCommand(
                commands,
                isLeft? TargetPositions.CORALSTATION_LEFT : TargetPositions.CORALSTATION_RIGHT
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

    public List<AutonSupplier> getAllCompRoutines() {
        return List.of(
            COMP_A_B(),
            COMP_H_A(),
            COMP_G_C_D_B(),
            COMP_I_K_L(),
            TEST_E_C()
        );
    }

    public record AutonSupplier(Supplier<AutoRoutine> routine, String name){}

    public record Auton(AutoRoutine routine, String name){}
}