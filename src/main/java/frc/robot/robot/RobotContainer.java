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
import java.util.Arrays;


public class RobotContainer {
    private final CommandSwerveDrivetrain swerve = CommandSwerveDrivetrain.getInstance();
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
        Controls.Driver.rotationResetTrigger.onTrue(
            swerve.resetPigeonCommand()
        );
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
