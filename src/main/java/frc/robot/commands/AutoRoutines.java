package frc.robot.commands;

import java.util.ArrayList;
import java.util.function.Supplier;
import java.util.stream.IntStream;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.scoring.ScoringSuperstructure;
import frc.robot.subsystems.scoring.ScoringSuperstructureState;

public class AutoRoutines {
    private final AutoFactory factory;

    public AutoRoutines(AutoFactory factory) {
        this.factory = factory;
    }

    public AutoRoutine auto1() {
        var pathName = "Path 1";
        var numPaths = 3;
        AutoRoutine routine = factory.newRoutine(pathName);

        ArrayList<AutoTrajectory> paths = new ArrayList<AutoTrajectory>();
        for (int i : IntStream.range(0, numPaths).toArray()) paths.add(routine.trajectory(pathName, i));

        routine.active().onTrue(Commands.sequence(
                paths.get(0).resetOdometry(),
                paths.get(0).cmd(),
                AutoCommands.L4Score.get(),
                paths.get(1).cmd(),
                AutoCommands.HPLoad.get(),
                paths.get(2).cmd(),
                AutoCommands.L4Score.get()
        ));
        return routine;
    }

    public AutoRoutine auto2() {
        var pathName = "Path 2";
        var numPaths = 3;
        AutoRoutine routine = factory.newRoutine(pathName);

        ArrayList<AutoTrajectory> paths = new ArrayList<AutoTrajectory>();
        for (int i : IntStream.range(0, numPaths).toArray()) paths.add(routine.trajectory(pathName, i));

        routine.active().onTrue(Commands.sequence(
                paths.get(0).resetOdometry(),
                paths.get(0).cmd(),
                AutoCommands.L4Score.get(),
                paths.get(1).cmd(),
                AutoCommands.HPLoad.get(),
                paths.get(2).cmd(),
                AutoCommands.L4Score.get()
        ));
        return routine;
    }

    public AutoRoutine auto3() {
        var pathName = "Path 3";
        var numPaths = 3;
        AutoRoutine routine = factory.newRoutine(pathName);

        ArrayList<AutoTrajectory> paths = new ArrayList<AutoTrajectory>();
        for (int i : IntStream.range(0, numPaths).toArray()) paths.add(routine.trajectory(pathName, i));

        routine.active().onTrue(Commands.sequence(
                paths.get(0).resetOdometry(),
                paths.get(0).cmd(),
                AutoCommands.L4Score.get(),
                paths.get(1).cmd(),
                AutoCommands.HPLoad.get(),
                paths.get(2).cmd(),
                AutoCommands.L4Score.get()
        ));
        return routine;
    }

    public AutoRoutine auto4() {
        var pathName = "Path 4";
        var numPaths = 3;
        AutoRoutine routine = factory.newRoutine(pathName);

        ArrayList<AutoTrajectory> paths = new ArrayList<AutoTrajectory>();
        for (int i : IntStream.range(0, numPaths).toArray()) paths.add(routine.trajectory(pathName, i));

        routine.active().onTrue(Commands.sequence(
                paths.get(0).resetOdometry(),
                paths.get(0).cmd(),
                AutoCommands.L4Score.get(),
                paths.get(1).cmd(),
                AutoCommands.HPLoad.get(),
                paths.get(2).cmd(),
                AutoCommands.L4Score.get()
        ));
        return routine;
    }
}