// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.FunctionalTrigger;
import frc.lib.util.DriverStationUtil;
import frc.robot.commands.auto.AutoRoutines;
import frc.robot.constants.Controls;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.climber.AbstractClimberSubsystem;
import frc.robot.subsystems.climber.ServoSubsystem;
import frc.robot.subsystems.drive.AbstractSwerveDrivetrain;
import frc.robot.subsystems.drive.DriveCommands;
import frc.robot.subsystems.drive.SwerveAutoRoutinesCreator;
import frc.robot.subsystems.led.LEDCommandFactory;
import frc.robot.subsystems.led.LEDStrip;
import frc.robot.subsystems.scoring.ScoringSuperstructure;
import frc.robot.subsystems.scoring.ScoringSuperstructureAction;
import frc.robot.subsystems.scoring.constants.ScoringConstants;

import static edu.wpi.first.units.Units.Meters;


public class RobotContainer {
    private final AbstractSwerveDrivetrain swerve = SubsystemManager.getInstance().getDrivetrain();
    private final ScoringSuperstructure scoringSuperstructure = SubsystemManager.getInstance().getScoringSuperstructure();
    private final ServoSubsystem servoSubsystem = SubsystemManager.getInstance().getServoSubsystem();
    private final AbstractClimberSubsystem climber = SubsystemManager.getInstance().getClimberSubsystem();
    @SuppressWarnings("unused")
    private final RobotSim robotSim = new RobotSim();
    @SuppressWarnings("unused")
    private final LEDStrip ledStrip = SubsystemManager.getInstance().getLEDStripSubsystem();
    private final ServoSubsystem servoSubsystem = SubsystemManager.getInstance().getServoSubsystem();
    private final AbstractClimberSubsystem climber = SubsystemManager.getInstance().getClimberSubsystem();

    private final StructArrayPublisher<Pose3d> componentPoses = NetworkTableInstance.getDefault()
        .getStructArrayTopic("zeroed component poses", Pose3d.struct).publish();

    public RobotContainer() {
        configureBindings();
        configureLEDs();
    }

    private void configureBindings() {
        swerve.setDefaultCommand(swerve.manualControl());    
        scoringSuperstructure.setDefaultCommand(scoringSuperstructure.runScoringState());

        DriverStationUtil.isRed.onChange(
                AutoRoutines.regenerateAutoRoutines()
                        .andThen(
                                Robot.replaceAutons()
                        )
        );

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
                scoringSuperstructure.useIntakeAction()
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
            // Controls.Operator.homingWristCommandTrigger.whileTrue(scoringSuperstructure.wristHomingCommand());
        }

        FunctionalTrigger.of(Controls.Driver.alignReefLeft)
            .whileTrue(() -> DriveCommands.moveToClosestReefPositionWithPID(FieldConstants.TargetPositions.Direction.LEFT, SubsystemManager.getInstance().getDrivetrain()::getPose));
        FunctionalTrigger.of(Controls.Driver.alignReefRight)
            .whileTrue(() -> DriveCommands.moveToClosestReefPositionWithPID(FieldConstants.TargetPositions.Direction.RIGHT, SubsystemManager.getInstance().getDrivetrain()::getPose));
        FunctionalTrigger.of(Controls.Driver.reefAlign)
            .and(Controls.Driver.alignReefLeft.negate())
            .and(Controls.Driver.alignReefRight.negate())
            .whileTrue(() -> DriveCommands.moveToClosestReefPositionWithPID(FieldConstants.TargetPositions.Direction.ALGAE, SubsystemManager.getInstance().getDrivetrain()::getPose));

        Controls.Driver.toggleAutoHeadingButton.onTrue(swerve.toggleAutoHeading());

        servoSubsystem.setDefaultCommand(servoSubsystem.stopServo());
        Controls.Driver.bindFunneltrigger.whileTrue(servoSubsystem.extendServo());
        Controls.Driver.dropFunnelTrigger.whileTrue(servoSubsystem.retractServo());
        Controls.Driver.climbTrigger.whileTrue(climber.climbCommand());
        Controls.Driver.prepClimbTrigger.whileTrue(climber.deClimbCommand());

        servoSubsystem.setDefaultCommand(servoSubsystem.stopServo());
        Controls.Driver.bindFunneltrigger.whileTrue(servoSubsystem.extendServo());
        Controls.Driver.dropFunnelTrigger.whileTrue(servoSubsystem.retractServo());
        Controls.Driver.climbTrigger.whileTrue(climber.climbCommand());
        Controls.Driver.prepClimbTrigger.whileTrue(climber.deClimbCommand());
        climber.setDefaultCommand(climber.testClimbCommand(Controls.Driver.testClimbSpeedSupplier));
        

        // OI.getInstance().driverController().Y_BUTTON.whileTrue(
        //         ElevatorSysID.sysIdQuasistatic(SysIdRoutine.Direction.kForward)
        // );
        // OI.getInstance().driverController().A_BUTTON.whileTrue(
        //         ElevatorSysID.sysIdQuasistatic(SysIdRoutine.Direction.kReverse)
        // );
        // OI.getInstance().driverController().POV_UP.whileTrue(
        //         ElevatorSysID.sysIdDynamic(SysIdRoutine.Direction.kForward)
        // );
        // OI.getInstance().driverController().POV_DOWN.whileTrue(
        //         ElevatorSysID.sysIdDynamic(SysIdRoutine.Direction.kReverse)
        // );
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

    public void configureLEDs() {
        ledStrip.setDefaultCommand(LEDCommandFactory.defaultCommand());
        new Trigger(scoringSuperstructure.getEndEffectorSubsystem()::hasCoral).onTrue(LEDCommandFactory.onCollectCoral());
    }
}
