package frc.robot.subsystems.climber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ServoSubsystem extends SubsystemBase {
    Servo servo = new Servo(ClimberConstants.SERVO_ID);

    public ServoSubsystem() {
        servo.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);
    }

    public Command setServoPosition(double position) {
        return run(() -> {
                    servo.setPosition(position);
                });
    }

    public double getServoPosition() {
        return servo.getPosition();
    }
}
