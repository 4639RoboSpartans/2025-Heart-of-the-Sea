package frc.robot.subsystems.climber;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.PackagePrivate;

import java.util.function.DoubleSupplier;

@PackagePrivate
class ConcreteClimberSubsystem extends ClimberSubsystem {
    @SuppressWarnings("FieldCanBeLocal")
    private final SparkFlex leftPivot, rightPivot;

    // TODO: add PIDs

    @PackagePrivate
    ConcreteClimberSubsystem() {
        leftPivot = new SparkFlex(ClimberConstants.IDs.LEFT_CLIMBER_MOTOR, SparkLowLevel.MotorType.kBrushless);
        rightPivot = new SparkFlex(ClimberConstants.IDs.RIGHT_CLIMBER_MOTOR, SparkLowLevel.MotorType.kBrushless);

        SparkBaseConfig cfg = new SparkFlexConfig()
            .idleMode(SparkBaseConfig.IdleMode.kBrake)
            .smartCurrentLimit(30); // TODO: use a better value

        leftPivot.configure(cfg, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
        rightPivot.configure(cfg, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    }


    @Override
    public Command runClimber(DoubleSupplier speedSupplier) {
        return run(
            () -> setMotors(speedSupplier.getAsDouble())
        ).finallyDo(
            () -> setMotors(0)
        );
    }

    private void setMotors(double speedSupplier) {
        leftPivot.set(speedSupplier);
        rightPivot.set(speedSupplier);
    }
}
