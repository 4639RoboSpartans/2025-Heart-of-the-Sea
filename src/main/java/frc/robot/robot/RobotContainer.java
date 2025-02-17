// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.robot;

import choreo.auto.AutoRoutine;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.AutoRoutines;
import frc.robot.constants.Controls;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.drive.DriveCommands;
import frc.robot.subsystems.drive.AbstractSwerveDrivetrain;
import frc.robot.subsystems.drive.SwerveAutoRoutinesCreator;
import frc.robot.subsystems.scoring.ScoringSuperstructure;
import frc.robot.subsystems.scoring.ScoringSuperstructureState;
import frc.robot.subsystems.scoring.constants.ScoringConstants;
import frc.robot.subsystems.vision.VisionSubsystem;

import java.util.Arrays;

import static edu.wpi.first.units.Units.Meters;


public class RobotContainer {
    private final AbstractSwerveDrivetrain swerve = SubsystemManager.getInstance().getDrivetrain();
    private final ScoringSuperstructure scoringSuperstructure = SubsystemManager.getInstance().getScoringSuperstructure();
    @SuppressWarnings("unused")
    private final RobotSim robotSim = new RobotSim();
    private final SendableChooser<Command> autoChooser;
    private final SendableChooser<Pose2d> startPositionChooser = new SendableChooser<>();

    private StructArrayPublisher<Pose3d> componentPoses = NetworkTableInstance.getDefault()
        .getStructArrayTopic("zeroed component poses", Pose3d.struct).publish();

    public RobotContainer() {

        // create auto routines here because we're configuring AutoBuilder in this method
        //TODO: take this out when we correctly refactor configurAutoBuilder to a new place
        AutoRoutines swerveAutoRoutines = SwerveAutoRoutinesCreator.createAutoRoutines(swerve);

        configureBindings();

        autoChooser = new SendableChooser<>();
        addAllCompAutons(autoChooser, swerveAutoRoutines);
        SmartDashboard.putData("Auto Chooser", autoChooser);

        startPositionChooser.setDefaultOption("DEFAULT", new Pose2d());
        SmartDashboard.putBoolean("pigeon reset", false);
        Arrays.stream(FieldConstants.AutonStartingPositions.values()).forEach(
            position -> startPositionChooser.addOption(position.name(), position.Pose)
        );
        SmartDashboard.putData("Selected Reset Position", startPositionChooser);
    }

    private void configureBindings() {
        swerve.setDefaultCommand(swerve.manualControl());
        scoringSuperstructure.setDefaultCommand(scoringSuperstructure.runScoringState());

        //Scoring Controls
        {
            Controls.Driver.rotationResetTrigger.onTrue(
                swerve.resetPigeon()
            );

            Controls.Operator.BargeScoringTrigger.onTrue(
                scoringSuperstructure.setScoringState(
                    ScoringSuperstructureState.BARGE_SCORING
                )
            );
            Controls.Operator.HPLoadingTrigger.onTrue(
                scoringSuperstructure.setScoringState(
                    ScoringSuperstructureState.HP_LOADING
                )
            );
            Controls.Operator.L1Trigger.onTrue(
                scoringSuperstructure.setScoringState(
                    ScoringSuperstructureState.L1
                )
            );
            Controls.Operator.L2Trigger.onTrue(
                scoringSuperstructure.setScoringState(
                    ScoringSuperstructureState.L2
                )
            );
            Controls.Operator.L3Trigger.onTrue(
                scoringSuperstructure.setScoringState(
                    ScoringSuperstructureState.L3
                )
            );
            Controls.Operator.L4Trigger.onTrue(
                scoringSuperstructure.setScoringState(
                    ScoringSuperstructureState.L4
                )
            );
            Controls.Operator.L2AlgaeTrigger.onTrue(
                scoringSuperstructure.setScoringState(
                    ScoringSuperstructureState.L2_ALGAE
                )
            );
            Controls.Operator.L3AlgaeTrigger.onTrue(
                scoringSuperstructure.setScoringState(
                    ScoringSuperstructureState.L3_ALGAE
                )
            );
            Controls.Operator.HoldTrigger.onTrue(
                scoringSuperstructure.hold()
            );

            Controls.Operator.ToggleManualControlTrigger.whileTrue(
                scoringSuperstructure.toggleManualControl()
            );
        }

        //Driving Controls
        {
            Controls.Driver.PathfindReef_0.whileTrue(
                DriveCommands.pathfindToReefCommand(
                    FieldConstants.TargetPositions.REEF_0
                )
            );
            Controls.Driver.PathfindReef_1.whileTrue(
                DriveCommands.pathfindToReefCommand(
                    FieldConstants.TargetPositions.REEF_1
                )
            );
            Controls.Driver.PathfindReef_2.whileTrue(
                DriveCommands.pathfindToReefCommand(
                    FieldConstants.TargetPositions.REEF_2
                )
            );
            Controls.Driver.PathfindReef_3.whileTrue(
                DriveCommands.pathfindToReefCommand(
                    FieldConstants.TargetPositions.REEF_3
                )
            );
            Controls.Driver.PathfindReef_4.whileTrue(
                DriveCommands.pathfindToReefCommand(
                    FieldConstants.TargetPositions.REEF_4
                )
            );
            Controls.Driver.PathfindReef_5.whileTrue(
                DriveCommands.pathfindToReefCommand(
                    FieldConstants.TargetPositions.REEF_5
                )
            );
        }
        /*OI.getInstance().operatorController().Y_BUTTON.whileTrue(
                ElevatorSysID.sysIdQuasistatic(SysIdRoutine.Direction.kForward)
        );
        OI.getInstance().operatorController().A_BUTTON.whileTrue(
                ElevatorSysID.sysIdQuasistatic(SysIdRoutine.Direction.kReverse)
        );
        OI.getInstance().operatorController().POV_UP.whileTrue(
                ElevatorSysID.sysIdDynamic(SysIdRoutine.Direction.kForward)
        );
        OI.getInstance().operatorController().POV_DOWN.whileTrue(
                ElevatorSysID.sysIdDynamic(SysIdRoutine.Direction.kReverse)
        );*/

    }

    private void addAllCompAutons(SendableChooser<Command> autoChooser, AutoRoutines swerveAutoRoutines) {
        for (AutoRoutine a : swerveAutoRoutines.getAllCompRoutines()) {
            autoChooser.addOption(a.toString(), a.cmd());
        }
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public void add3DComponentPoses() {
        componentPoses.set(
            new Pose3d[]{
                new Pose3d(
                    new Translation3d(
                        0,
                        0,
                        (scoringSuperstructure.getCurrentElevatorLength().in(Meters)
                            - ScoringConstants.ElevatorConstants.STARTING_HEIGHT.in(Meters)) / 3.0),
                    new Rotation3d()
                ),
                new Pose3d(
                    new Translation3d(
                        0,
                        0,
                        (scoringSuperstructure.getCurrentElevatorLength().in(Meters)
                            - ScoringConstants.ElevatorConstants.STARTING_HEIGHT.in(Meters)) * 2.0 / 3.0),
                    new Rotation3d()
                ),
                new Pose3d(
                    new Translation3d(
                        0,
                        0,
                        (scoringSuperstructure.getCurrentElevatorLength().in(Meters)
                            - ScoringConstants.ElevatorConstants.STARTING_HEIGHT.in(Meters))),
                    new Rotation3d()
                ),
                new Pose3d(
                    new Translation3d(
                        0,
                        0,
                        (scoringSuperstructure.getCurrentElevatorLength().in(Meters)
                            - ScoringConstants.ElevatorConstants.STARTING_HEIGHT.in(Meters))
                    ).plus(
                        ScoringConstants.HopperConstants.Hopper3DSimOffset
                    ),
                    new Rotation3d(
                        0,
                        -scoringSuperstructure.getCurrentWristRotation().minus(ScoringConstants.HopperConstants.IDLE_ROTATION).getRadians(),
                        0
                    )
                )
            }
        );
    }
}
