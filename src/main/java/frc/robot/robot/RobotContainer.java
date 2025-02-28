// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.lib.FunctionalTrigger;
import frc.lib.led.*;
import frc.robot.commands.AutoCommands;
import frc.robot.commands.AutoRoutines;
import frc.robot.commands.AutoRoutines.Auton;
import frc.robot.constants.Controls;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.drive.AbstractSwerveDrivetrain;
import frc.robot.subsystems.drive.DriveCommands;
import frc.robot.subsystems.drive.SwerveAutoRoutinesCreator;
import frc.robot.subsystems.scoring.ScoringSuperstructure;
import frc.robot.subsystems.scoring.ScoringSuperstructureAction;
import frc.robot.subsystems.scoring.constants.ScoringConstants;

import java.util.Arrays;

import static edu.wpi.first.units.Units.Meters;


public class RobotContainer {
    private final AbstractSwerveDrivetrain swerve = SubsystemManager.getInstance().getDrivetrain();
    private final ScoringSuperstructure scoringSuperstructure = SubsystemManager.getInstance().getScoringSuperstructure();
    @SuppressWarnings("unused")
    private final RobotSim robotSim = new RobotSim();
    private final SendableChooser<Command> autoChooser;
    private final SendableChooser<Pose2d> startPositionChooser = new SendableChooser<>();
    @SuppressWarnings("unused")
    private final LEDStrip ledStrip = SubsystemManager.getInstance().getLEDStripSubsystem();

    private final StructArrayPublisher<Pose3d> componentPoses = NetworkTableInstance.getDefault()
        .getStructArrayTopic("zeroed component poses", Pose3d.struct).publish();

    public RobotContainer() {

        // create auto routines here because we're configuring AutoBuilder in this method
        //TODO: take this out when we correctly refactor configureAutoBuilder to a new place
        AutoRoutines swerveAutoRoutines = SwerveAutoRoutinesCreator.createAutoRoutines(swerve);

        configureBindings();

        autoChooser = new SendableChooser<>();
        addAllCompAutons(autoChooser, swerveAutoRoutines);
        autoChooser.addOption("TEST", AutoCommands.L4Score.get());
        SmartDashboard.putData("Auto Chooser", autoChooser);

        startPositionChooser.setDefaultOption("DEFAULT", new Pose2d());
        SmartDashboard.putBoolean("pigeon reset", false);
        Arrays.stream(FieldConstants.AutonStartingPositions.values()).forEach(
            position -> startPositionChooser.addOption(position.name(), position.Pose)
        );
        SmartDashboard.putData("Selected Reset Position", startPositionChooser);

        configureLED();
    }

    private void configureBindings() {
        swerve.setDefaultCommand(swerve.manualControl());
        scoringSuperstructure.setDefaultCommand(scoringSuperstructure.runScoringState());

        //Scoring Controls
        {
            Controls.Driver.rotationResetTrigger.whileTrue(
                swerve.resetHeadingToZero()
            );

            Controls.Driver.BargeScoringTrigger.onTrue(
                scoringSuperstructure.setAction(
                    ScoringSuperstructureAction.SCORE_BARGE
                )
            );
            Controls.Operator.HPLoadingTrigger.onTrue(
                scoringSuperstructure.setAction(
                    ScoringSuperstructureAction.INTAKE_FROM_HP
                )
            );
            Controls.Driver.ProcessorTrigger.onTrue(
                scoringSuperstructure.setAction(
                    ScoringSuperstructureAction.SCORE_PROCESSOR
                )
            );
            Controls.Operator.L1Trigger.onTrue(
                scoringSuperstructure.setAction(
                    ScoringSuperstructureAction.SCORE_L1_CORAL
                )
            );
            Controls.Operator.L2Trigger.onTrue(
                scoringSuperstructure.setAction(
                    ScoringSuperstructureAction.SCORE_L2_CORAL
                )
            );
            Controls.Operator.L3Trigger.onTrue(
                scoringSuperstructure.setAction(
                    ScoringSuperstructureAction.SCORE_L3_CORAL
                )
            );
            Controls.Operator.L4Trigger.onTrue(
                scoringSuperstructure.setAction(
                    ScoringSuperstructureAction.SCORE_L4_CORAL
                )
            );
            Controls.Driver.L2AlgaeTrigger.onTrue(
                scoringSuperstructure.setAction(
                    ScoringSuperstructureAction.INTAKE_L2_ALGAE
                )
            );
            Controls.Driver.L3AlgaeTrigger.onTrue(
                scoringSuperstructure.setAction(
                    ScoringSuperstructureAction.INTAKE_L3_ALGAE
                )
            );
            Controls.Operator.ScoringIdleTrigger.onTrue(
                scoringSuperstructure.setAction(
                    ScoringSuperstructureAction.IDLE
                )
            );

            Controls.Operator.ToggleManualControlTrigger.whileTrue(
                scoringSuperstructure.toggleManualControl()
            );

            Controls.Operator.HomingCommandTrigger.whileTrue(scoringSuperstructure.elevatorHomingCommand());
        }

        FunctionalTrigger.of(Controls.Driver.alignReefLeft)
            .whileTrue(() -> DriveCommands.moveToClosestReefPositionWithTransformation((byte) 0));
        FunctionalTrigger.of(Controls.Driver.alignReefRight)
            .whileTrue(() -> DriveCommands.moveToClosestReefPositionWithTransformation((byte) 1));
        FunctionalTrigger.of(Controls.Driver.reefAlign)
            .and(Controls.Driver.alignReefLeft.negate())
            .and(Controls.Driver.alignReefRight.negate())
            .whileTrue(() -> DriveCommands.moveToClosestReefPositionWithTransformation((byte) 2));

        FunctionalTrigger.of(Controls.Driver.processorAlign)
                .whileTrue(DriveCommands::moveToProcessor);
//        FunctionalTrigger.of(Controls.Driver.coralStationAlign)
//            .and(Controls.Driver.targetLeft).whileTrue(() -> DriveCommands.moveToDesiredCoralStationPosition(true));
//        FunctionalTrigger.of(Controls.Driver.coralStationAlign)
//            .and(Controls.Driver.targetRight).whileTrue(() -> DriveCommands.moveToDesiredCoralStationPosition(false));


        // OI.getInstance().driverController().Y_BUTTON.whileTrue(
        //         DriveSysID.sysIdQuasistatic(SysIdRoutine.Direction.kForward)
        // );
        // OI.getInstance().driverController().A_BUTTON.whileTrue(
        //         DriveSysID.sysIdQuasistatic(SysIdRoutine.Direction.kReverse)
        // );
        // OI.getInstance().driverController().POV_UP.whileTrue(
        //         DriveSysID.sysIdDynamic(SysIdRoutine.Direction.kForward)
        // );
        // OI.getInstance().driverController().POV_DOWN.whileTrue(
        //         DriveSysID.sysIdDynamic(SysIdRoutine.Direction.kReverse)
        // );

        // OI.getInstance().driverController().A_BUTTON.onTrue(MiscellaneousCommands.ElevatorUpDownTest());
    }

    private void addAllCompAutons(SendableChooser<Command> autoChooser, AutoRoutines swerveAutoRoutines) {
        for (Auton a : swerveAutoRoutines.getAllCompRoutines()) {
            autoChooser.addOption(a.name(), a.routine().cmd());
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
                        ScoringConstants.EndEffectorConstants.Hopper3DSimOffset
                    ),
                    new Rotation3d(
                        0,
                        -scoringSuperstructure.getCurrentWristRotation().minus(ScoringConstants.EndEffectorConstants.IDLE_ROTATION).getRadians(),
                        0
                    )
                )
            }
        );
    }
    /*
        Intake coral - triple flash green -> solid green
        Honed -> triple flash green
        Not honed -> flash red
        Default disabled ->breathing alliance color
        Reef align -> flashing orange blue
        Default -> blue orange cycling
     */
    private void configureLED() {
        LEDPattern idlePattern = (ledIdx, time) -> {
            time *= 2;
            double x = ledIdx * 0.2 + time * 3;
            double h = 30 * Math.pow(Math.sin(x), 2) + 90;
            double v = Math.pow(Math.sin(time), 2) * 0.9 + 0.1;

            return new Color8Bit(Color.fromHSV((int) (h), 255, (int) (255 * v)));
        };
        LEDPattern transitionPattern = new BasicLEDPattern(3,
            new Color8Bit(Color.kGreen)
        );
        LEDPattern elevatorMovePattern = new BasicLEDPattern(3,
            new Color8Bit(Color.kYellow)
        );
        LEDPattern executingActionPattern = new SolidLEDPattern(new Color8Bit(255, 0, 0));

        ledStrip.setDefaultCommand(new RunCommand(() -> {
            ledStrip.usePattern(
                scoringSuperstructure.getCurrentAction() == ScoringSuperstructureAction.IDLE ?
                    idlePattern :
                    switch (scoringSuperstructure.getCurrentState()) {
                        case TRANSITION_BEFORE_ELEVATOR -> transitionPattern;
                        case ELEVATOR_MOVE_WITH_TRANSITION -> elevatorMovePattern;
                        case TRANSITION_AFTER_ELEVATOR -> transitionPattern;
                        case ELEVATOR_MOVE_NO_TRANSITION -> elevatorMovePattern;
                        case EXECUTING_ACTION -> executingActionPattern;
                        case DONE -> idlePattern;
                    }
            );
        }, ledStrip));
    }
}
