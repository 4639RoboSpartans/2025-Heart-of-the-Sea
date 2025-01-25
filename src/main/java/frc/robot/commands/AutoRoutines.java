package frc.robot.commands;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;

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
        AutoTrajectory testPath1 = routine.trajectory("Test Path 1");

        // When the routine starts, reset odometry, shoot the first gamepiece, then go to the "C2" location
        routine.active().onTrue(
                Commands.sequence(
                        testPath1.resetOdometry(),
                        testPath1.cmd()
                )
        );

        // TODO: This giant block of commented-out-code is messy. Fix it. Either justify commenting out or remove it.
        //  -- Jonathan

//        // Pick up and shoot the gamepiece at the "C2" location, then go to the "M1" location
//        startToC2.active().whileTrue(intakeSubsystem.intake());
//        startToC2.done().onTrue(shooterSubsystem.shoot().andThen(C2toM1.cmd()));
//
//        // Run the intake when we are approaching a gamepiece
//        routine.anyActive(C2toM1, scoreToM2, scoreToM3, M1toM2, M2toM3) //
//                .whileTrue(intakeSubsystem.intake());

        // If we picked up the gamepiece, go score, then go to the next midline location
        // If we didn't pick up the gamepiece, go directly to the next midline location

//        // M1
//        Trigger atM1 = C2toM1.done();
//        atM1.and(shooterSubsystem::noGamepiece).onTrue(M1toM2.cmd());
//        atM1.and(shooterSubsystem::hasGamepiece).onTrue(M1toScore.cmd());
//        M1toScore.done().onTrue(shooterSubsystem.shoot().andThen(scoreToM2.cmd()));
//
//        // M2
//        Trigger atM2 = routine.anyDone(scoreToM2, M1toM2); //
//        atM2.and(shooterSubsystem::noGamepiece).onTrue(M2toM3.cmd());
//        atM2.and(shooterSubsystem::hasGamepiece).onTrue(M2toScore.cmd());
//        M2toScore.done().onTrue(shooterSubsystem.shoot().andThen(scoreToM3.cmd()));
//
//        // M3
//        Trigger atM3 = routine.anyDone(scoreToM3, M2toM3);
//        atM3.and(shooterSubsystem::hasGamepiece).onTrue(M3toScore.cmd());
//        M3toScore.done().onTrue(shooterSubsystem.shoot());

        return routine;
    }

    public AutoRoutine testPath2() {
        AutoRoutine routine = factory.newRoutine("Test Path 2");

        // This routine uses segments between pre-defined handoff points.
        // Expand the tooltip for information about their naming convention ->

        // Load the routine's trajectories
        AutoTrajectory testPath2 = routine.trajectory("Test Path 2");

        // When the routine starts, reset odometry, shoot the first gamepiece, then go to the "C2" location
        routine.active().onTrue(
                Commands.sequence(
                        testPath2.resetOdometry(),
                        testPath2.cmd()
                )
        );

        // TODO: This giant block of commented-out-code is messy. Fix it. Either justify commenting out or remove it.
        //  -- Jonathan

//        // Pick up and shoot the gamepiece at the "C2" location, then go to the "M1" location
//        startToC2.active().whileTrue(intakeSubsystem.intake());
//        startToC2.done().onTrue(shooterSubsystem.shoot().andThen(C2toM1.cmd()));
//
//        // Run the intake when we are approaching a gamepiece
//        routine.anyActive(C2toM1, scoreToM2, scoreToM3, M1toM2, M2toM3) //
//                .whileTrue(intakeSubsystem.intake());

        // If we picked up the gamepiece, go score, then go to the next midline location
        // If we didn't pick up the gamepiece, go directly to the next midline location

//        // M1
//        Trigger atM1 = C2toM1.done();
//        atM1.and(shooterSubsystem::noGamepiece).onTrue(M1toM2.cmd());
//        atM1.and(shooterSubsystem::hasGamepiece).onTrue(M1toScore.cmd());
//        M1toScore.done().onTrue(shooterSubsystem.shoot().andThen(scoreToM2.cmd()));
//
//        // M2
//        Trigger atM2 = routine.anyDone(scoreToM2, M1toM2); //
//        atM2.and(shooterSubsystem::noGamepiece).onTrue(M2toM3.cmd());
//        atM2.and(shooterSubsystem::hasGamepiece).onTrue(M2toScore.cmd());
//        M2toScore.done().onTrue(shooterSubsystem.shoot().andThen(scoreToM3.cmd()));
//
//        // M3
//        Trigger atM3 = routine.anyDone(scoreToM3, M2toM3);
//        atM3.and(shooterSubsystem::hasGamepiece).onTrue(M3toScore.cmd());
//        M3toScore.done().onTrue(shooterSubsystem.shoot());

        return routine;
    }
}