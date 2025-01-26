package frc.robot.commands;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;

public class AutoRoutines {
    private final AutoFactory factory;

    public AutoRoutines(AutoFactory factory) {
        this.factory = factory;
    }

    public AutoRoutine testPath1() {
        AutoRoutine routine = factory.newRoutine("Test Path 1");

        // This routine uses segments between pre-defined handoff points.
        // Expand the tooltip for information about their naming convention ->

        // Load the routine's trajectories
        AutoTrajectory testPath1_0 = routine.trajectory("Test Path 1", 0);
        AutoTrajectory testPath1_1 = routine.trajectory("Test Path 1", 1);
        AutoTrajectory testPath1_2 = routine.trajectory("Test Path 1", 2);
        AutoTrajectory testPath1_3 = routine.trajectory("Test Path 1", 3);

        // When the routine starts, reset odometry, shoot the first gamepiece, then go to the "C2" location
        routine.active().onTrue(
                Commands.sequence(
                        testPath1_0.resetOdometry(),
                        testPath1_0.cmd(),
                        CommandSwerveDrivetrain.getInstance().stopCommand().withTimeout(1),
                        testPath1_1.cmd(),
                        CommandSwerveDrivetrain.getInstance().stopCommand().withTimeout(1),
                        testPath1_2.cmd(),
                        CommandSwerveDrivetrain.getInstance().stopCommand().withTimeout(1),
                        testPath1_3.cmd()
                )
        );
        return routine;
    }
}