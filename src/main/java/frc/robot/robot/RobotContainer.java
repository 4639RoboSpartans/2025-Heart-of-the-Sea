// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Controls;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.scoring.ScoringSuperstructure;
import frc.robot.subsystems.scoring.ScoringSuperstructureState;

import java.util.Arrays;


public class RobotContainer {
    private final CommandSwerveDrivetrain swerve = CommandSwerveDrivetrain.getInstance();
    private final ScoringSuperstructure scoringSuperstructure = ScoringSuperstructure.getInstance();
    @SuppressWarnings("unused")
    private final RobotSim robotSim = new RobotSim();
    private final SendableChooser<Command> autoChooser;
    private final SendableChooser<Pose2d> m_startPositionChooser = new SendableChooser<>();

    public RobotContainer() {
        configureBindings();

        autoChooser = new SendableChooser<>();
        autoChooser.addOption("Auto 1", swerve.getAutoRoutines().auto1().cmd());
        autoChooser.addOption("Auto 2", swerve.getAutoRoutines().auto2().cmd());
        autoChooser.addOption("Auto 3", swerve.getAutoRoutines().auto3().cmd());
        autoChooser.addOption("Auto 4", swerve.getAutoRoutines().auto4().cmd());
        SmartDashboard.putData("Auto Chooser", autoChooser);

        m_startPositionChooser.setDefaultOption("DEFAULT", new Pose2d());
        SmartDashboard.putBoolean("pigeon reset", false);
        Arrays.stream(FieldConstants.AutonStartingPositions.values()).forEach(
                position -> m_startPositionChooser.addOption(position.name(), position.Pose)
        );
        SmartDashboard.putData("Selected Reset Position", m_startPositionChooser);
    }

    private void configureBindings() {
        swerve.setDefaultCommand(swerve.applyRequest(swerve::fieldCentricRequestSupplier));
        scoringSuperstructure.setDefaultCommand(scoringSuperstructure.runScoringState());

        Controls.Driver.rotationResetTrigger.onTrue(
            swerve.resetPigeonCommand()
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
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
