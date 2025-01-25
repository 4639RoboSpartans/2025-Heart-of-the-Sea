// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.oi.OI;
import frc.robot.constants.Controls;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import java.util.Arrays;


public class RobotContainer {
    private final CommandSwerveDrivetrain swerve = CommandSwerveDrivetrain.getInstance();
    private final SendableChooser<Command> autoChooser;
    private final SendableChooser<Pose2d> m_startPositionChooser = new SendableChooser<>();

    public RobotContainer() {
        configureBindings();
        autoChooser = new SendableChooser<>();
        autoChooser.addOption("Test Path 1", swerve.getAutoRoutines().testPath1().cmd());
        autoChooser.addOption("Test Path 2", swerve.getAutoRoutines().testPath2().cmd());
        SmartDashboard.putData("Auto Chooser", autoChooser);
        m_startPositionChooser.setDefaultOption("DEFAULT", new Pose2d());
        Arrays.stream(FieldConstants.AutonStartingPositions.values()).forEach(
                position -> m_startPositionChooser.addOption(position.name(), position.Pose)
        );
        SmartDashboard.putData("Selected Reset Position", m_startPositionChooser);
    }

    private void configureBindings() {
        /*OI.getInstance().driverController().A_BUTTON.whileTrue(
            DriveSysID.sysIdQuasistatic(Direction.kForward)
        );
        OI.getInstance().driverController().B_BUTTON.whileTrue(
                DriveSysID.sysIdQuasistatic(Direction.kReverse)
        );
        OI.getInstance().driverController().X_BUTTON.whileTrue(
                DriveSysID.sysIdDynamic(Direction.kForward)
        );
        OI.getInstance().driverController().Y_BUTTON.whileTrue(
                DriveSysID.sysIdDynamic(Direction.kReverse)
        );*/
        swerve.setDefaultCommand(swerve.applyRequest(swerve::fieldCentricRequestSupplier));
        Controls.Driver.rotationResetTrigger.onTrue(
            swerve.resetPigeonCommand()
        );

        /** Resets Pose to desired pose set by dashboard */
        OI.getInstance().driverController().RIGHT_STICK.whileTrue(
            swerve.run(() -> 
                swerve.resetPose(
                    m_startPositionChooser.getSelected()
                )
            )
        );
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
