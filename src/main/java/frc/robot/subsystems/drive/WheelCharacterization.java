package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.drive.constants.TunerConstants;

public class WheelCharacterization {
    public static Command runWheelCharacterization(AbstractSwerveDrivetrain swerve) {
        return new InstantCommand(
            swerve::startWheelCharacterization
        ).andThen(
            swerve.rotateCommand()
                .withTimeout(
                    0.1
                )
        ).andThen(
            swerve.rotateCommand()
                .until(
                    swerve::isAtSameRotation
                )
        ).andThen(
            swerve.stop()
        );
    }
}
