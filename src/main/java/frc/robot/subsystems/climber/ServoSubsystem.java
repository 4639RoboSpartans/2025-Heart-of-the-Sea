package frc.robot.subsystems.climber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ServoSubsystem extends SubsystemBase {
    Servo servo = new Servo(ClimberConstants.SERVO_ID);
    double goal;

    //this is super scuffed but this is the only way to guarantee consistent movement with the servo
    public ProfiledPIDController controller = new ProfiledPIDController(0, 0, 0,
            new TrapezoidProfile.Constraints(3, 1.7) //values from tuesday test
    );

    public Command setServoPosition(double position) {
        return runOnce(() -> controller.setGoal(position))
                .andThen(() -> {
                    controller.calculate(servo.getPosition());
                    servo.setPosition(controller.getSetpoint().position);
                }).until(this::atGoal);
    }

    public Command runServoPosition(double position) {
        return runOnce(() -> controller.setGoal(position))
                .andThen(() -> {
                    controller.calculate(servo.getPosition());
                    servo.setPosition(controller.getSetpoint().position);
                });
    }

    public boolean atGoal() {
        return MathUtil.isNear(goal, servo.getPosition(), 0.05);
    }
}
