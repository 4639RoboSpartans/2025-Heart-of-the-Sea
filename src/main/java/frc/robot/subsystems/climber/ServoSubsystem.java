package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystemManager.SubsystemInstantiator;

import java.util.Objects;

public class ServoSubsystem extends SubsystemBase {
    public static SubsystemInstantiator<ServoSubsystem> createInstance() {
        return new SubsystemInstantiator<>(ServoSubsystem::new);
    }

    private final Servo funnelServo;

    public ServoSubsystem() {
        funnelServo = new Servo(8);
        funnelServo.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);
        funnelServo.setSpeed(1.0);
    }

    public Command retractServo() {
        return Commands.run(
                () -> funnelServo.setSpeed(-1.0),
                this
        );
    }

    public Command extendServo() {
        return Commands.run(
                () -> funnelServo.setSpeed(1.0),
                this
        );
    }

    public Command stopServo() {
        return Commands.run(
            () -> funnelServo.setSpeed(getServoPosition()),
            this
        );
    }

    public double getServoPosition() {
        return funnelServo.getPosition();
    }
}
