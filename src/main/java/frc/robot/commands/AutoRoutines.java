package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.IntStream;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.CommandsUtil;

public class AutoRoutines {
    private final AutoFactory factory;

    public AutoRoutines(AutoFactory factory) {
        this.factory = factory;
    }

    public AutoRoutine COMP_J_K() {
        return compileAuton(
                List.of('J', 'K'),
                List.of(4, 4),
                true
        );
    }

    public AutoRoutine COMP_H_G() {
        return compileAuton(
                List.of('H', 'G'),
                List.of(4, 4),
                true
        );
    }

    public AutoRoutine COMP_A_B() {
        return compileAuton(
                List.of('A', 'B'),
                List.of(4, 4),
                true
        );
    }

    public AutoRoutine COMP_H_A() {
        return compileAuton(
                List.of('H', 'A'),
                List.of(4, 4),
                true
        );
    }

    public AutoRoutine COMP_G_C_D_B() {
        return compileAuton(
                List.of('G', 'C', 'D', 'B'),
                List.of(4, 4, 4, 4),
                true
        );
    }

    public AutoRoutine TEST_A_B() {
        return compileAuton(
                List.of('A', 'B'),
                List.of(1, 2),
                false
        );
    }


    /**
     * @param scoringLocations all locations as designated in path naming
     * @param scoringHeights   all heights, each corresponding to a location,
     *                         must be the same size as {@param scoringLocations}
     * @param comp             is the auton for competition
     * @return a new routine with the specified characteristics
     */
    private AutoRoutine compileAuton(List<Character> scoringLocations, List<Integer> scoringHeights, boolean comp) {
        StringBuilder pathName = new StringBuilder(comp ? "COMP-" : "TEST-");
        StringBuilder name = new StringBuilder(pathName.toString());
        int numPaths = scoringHeights.size() * 2 - 1;
        int numScores = scoringLocations.size();
        for (int i = 0; i < numScores; i++) {
            pathName.append(scoringLocations.get(i));
            name.append(scoringLocations.get(i));
            name.append(scoringHeights.get(i));
            if (i < numScores - 1) {
                pathName.append("-");
                name.append("-");
            }
        }
        AutoRoutine routine = factory.newRoutine(name.toString());
        ArrayList<AutoTrajectory> paths = new ArrayList<>();
        for (int i : IntStream.range(0, numPaths).toArray()) {
            paths.add(routine.trajectory(pathName.toString(), i));
        }
        List<Command> commands = new ArrayList<>();
        commands.add(paths.get(0).resetOdometry());
        for (int i = 0; i < numPaths; i++) {
            commands.add(paths.get(i).cmd());
            commands.add(
                    i % 2 == 0 ?
                            switch (scoringHeights.get(i / 2)) {
                                case 1 -> AutoCommands.L1Score.get();
                                case 2 -> AutoCommands.L2Score.get();
                                case 3 -> AutoCommands.L3Score.get();
                                case 4 -> AutoCommands.L4Score.get();
                                default -> AutoCommands.HPLoad.get();
                            } : AutoCommands.HPLoad.get()
            );
        }
        routine.active().onTrue(
                CommandsUtil.sequence(commands)
        );
        return routine;
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