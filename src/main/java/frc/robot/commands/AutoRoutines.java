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
    private final Supplier<Command> oneSecondTimeout = () -> CommandSwerveDrivetrain.getInstance().stopCommand().withTimeout(1);

    public AutoRoutines(AutoFactory factory) {
        this.factory = factory;
    }

    public AutoRoutine Path1() {
        AutoRoutine routine = factory.newRoutine("Path 1");

        AutoTrajectory testPath1_0 = routine.trajectory("Path 1", 0);
        AutoTrajectory testPath1_1 = routine.trajectory("Path 1", 1);
        AutoTrajectory testPath1_2 = routine.trajectory("Path 1", 2);

        routine.active().onTrue(
                Commands.sequence(
                        ScoringSuperstructure.getInstance().setScoringState(ScoringSuperstructureState.HP_LOADING),
                        ScoringSuperstructure.getInstance().runScoringState()
                )
        );
        return routine;
    }

    public AutoRoutine auto1(){
        var pathName = "Path 1";
        var numPaths = 3;
        var resetOdometry = true;
        AutoRoutine routine = factory.newRoutine(pathName);

        ArrayList<AutoTrajectory> paths = new ArrayList<AutoTrajectory>();
        for(int i:IntStream.range(0, numPaths).toArray()) paths.add(routine.trajectory(pathName, i));

        routine.active().onTrue(Commands.sequence(
            resetOdometry ? paths.get(0).resetOdometry() : Commands.none(),
            paths.get(0).cmd(),
            oneSecondTimeout.get(),
            paths.get(1).cmd(),
            oneSecondTimeout.get(),
            paths.get(2).cmd()
        ));
        return routine;
    }

    public AutoRoutine auto2(){
        var pathName = "Path 2";
        var numPaths = 3;
        var resetOdometry = true;
        AutoRoutine routine = factory.newRoutine(pathName);

        ArrayList<AutoTrajectory> paths = new ArrayList<AutoTrajectory>();
        for(int i:IntStream.range(0, numPaths).toArray()) paths.add(routine.trajectory(pathName, i));

        routine.active().onTrue(Commands.sequence(
            resetOdometry ? paths.get(0).resetOdometry() : Commands.none(),
            paths.get(0).cmd(),
            oneSecondTimeout.get(),
            paths.get(1).cmd(),
            oneSecondTimeout.get(),
            paths.get(2).cmd()
        ));
        return routine;
    }

    public AutoRoutine auto3(){
        var pathName = "Path 3";
        var numPaths = 3;
        var resetOdometry = true;
        AutoRoutine routine = factory.newRoutine(pathName);

        ArrayList<AutoTrajectory> paths = new ArrayList<AutoTrajectory>();
        for(int i:IntStream.range(0, numPaths).toArray()) paths.add(routine.trajectory(pathName, i));

        routine.active().onTrue(Commands.sequence(
            resetOdometry ? paths.get(0).resetOdometry() : Commands.none(),
            paths.get(0).cmd(),
            oneSecondTimeout.get(),
            paths.get(1).cmd(),
            oneSecondTimeout.get(),
            paths.get(2).cmd()
        ));
        return routine;
    }

    public AutoRoutine auto4(){
        var pathName = "Path 4";
        var numPaths = 3;
        var resetOdometry = true;
        AutoRoutine routine = factory.newRoutine(pathName);

        ArrayList<AutoTrajectory> paths = new ArrayList<AutoTrajectory>();
        for(int i:IntStream.range(0, numPaths).toArray()) paths.add(routine.trajectory(pathName, i));

        routine.active().onTrue(Commands.sequence(
            resetOdometry ? paths.get(0).resetOdometry() : Commands.none(),
            paths.get(0).cmd(),
            oneSecondTimeout.get(),
            paths.get(1).cmd(),
            oneSecondTimeout.get(),
            paths.get(2).cmd()
        ));
        return routine;
    }

    
}