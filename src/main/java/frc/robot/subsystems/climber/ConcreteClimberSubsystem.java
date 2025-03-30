package frc.robot.subsystems.climber;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.Servo;

import java.util.Objects;

public class ConcreteClimberSubsystem extends AbstractClimberSubsystem {
    SparkMax climberMotor;
    ClimberState climberState = ClimberState.STOWED;
    AbsoluteEncoder encoder;

    private static volatile ConcreteClimberSubsystem instance;

    public static synchronized ConcreteClimberSubsystem getInstance() {
        return instance = Objects.requireNonNullElseGet(instance, ConcreteClimberSubsystem::new);
    }

    public ConcreteClimberSubsystem() {
        init();
        this.climberMotor = new SparkMax(ClimberConstants.CLIMBER_ID, SparkLowLevel.MotorType.kBrushed);
        this.encoder = climberMotor.getAbsoluteEncoder();
    }

    @Override
    void setClimberSpeed(double speed) {
        climberMotor.set(speed);
    }

    @Override
    ClimberState getClimberState() {
        return climberState;
    }

    @Override
    void setClimberState(ClimberState state) {
        climberState = state;
    }

    @Override
    double getEncoderPosition() {
        return encoder.getPosition();
    }
}