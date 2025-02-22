package frc.robot.subsystems.climber;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.annotation.PackagePrivate;
import frc.robot.subsystems.climber.ClimberConstants.IDs;

import java.util.function.DoubleSupplier;

@PackagePrivate
class ConcreteClimberSubsystem extends AbstractClimberSubsystem {
    private final MotorController motor;

    public ConcreteClimberSubsystem() {
        SparkFlex motor = new SparkFlex(IDs.CLIMBER_MOTOR, SparkLowLevel.MotorType.kBrushless);
        SparkBaseConfig cfg = new SparkFlexConfig()
            .idleMode(SparkBaseConfig.IdleMode.kBrake)
            .smartCurrentLimit(30);
        motor.configure(cfg, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

        this.motor = motor;
//        this.motor = new RatchetMotor(motor, IDs.CLIMBER_RATCHET_DIO_PORT, RatchetMotor.RatchetDirection.Forwards);
    }

    @Override
    public Command runClimber(DoubleSupplier speedSupplier) {
        return run(
            () -> setMotor(speedSupplier.getAsDouble())
        ).finallyDo(
            () -> setMotor(0)
        );
    }

    private void setMotor(double speed) {
        motor.set(speed);
    }
}
