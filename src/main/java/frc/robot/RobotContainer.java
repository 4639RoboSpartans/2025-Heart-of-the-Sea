// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.lib.oi.OI;
import frc.robot.constants.Controls;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.DriveSysID;

import java.util.Arrays;


public class RobotContainer {
    private final OI oi = OI.getInstance();
    private final CommandSwerveDrivetrain swerve = CommandSwerveDrivetrain.getInstance();
    private final SendableChooser<Command> autoChooser;
    private final SendableChooser<Pose2d> m_startPositionChooser = new SendableChooser<>();

    public RobotContainer() {
        configureBindings();
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData(autoChooser);
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
