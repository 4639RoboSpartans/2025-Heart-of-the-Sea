package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.SubsystemManager;

import java.util.Objects;
import java.util.function.DoubleSupplier;

public class ServoTestSubsystem extends SubsystemBase {
    private static ServoTestSubsystem instance;

    public static ServoTestSubsystem getInstance(SubsystemManager.GetInstanceAccess getInstanceAccess) {
        return Objects.requireNonNullElseGet(instance, ServoTestSubsystem::new);
    }

    private final Servo funnelServo = new Servo(ClimberConstants.SERVO_ID);
    {
        funnelServo.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);
    }

    public Command runServo(DoubleSupplier pos) {
        return run(
                () -> funnelServo.setSpeed(pos.getAsDouble())
        );
    }
}
