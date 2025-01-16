// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.lib.oi.OI;
import frc.lib.oi.OI;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;


public class RobotContainer {
    private final OI oi = OI.getInstance();
    private final CommandSwerveDrivetrain swerve = CommandSwerveDrivetrain.getInstance();

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        OI.getInstance().driverController().A_BUTTON.whileTrue(
            swerve.sysIdQuasistatic(Direction.kForward)
        );
        OI.getInstance().driverController().B_BUTTON.whileTrue(
            swerve.sysIdQuasistatic(Direction.kReverse)
        );
        OI.getInstance().driverController().X_BUTTON.whileTrue(
            swerve.sysIdDynamic(Direction.kForward)
        );
        OI.getInstance().driverController().Y_BUTTON.whileTrue(
            swerve.sysIdDynamic(Direction.kReverse)
        );
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
