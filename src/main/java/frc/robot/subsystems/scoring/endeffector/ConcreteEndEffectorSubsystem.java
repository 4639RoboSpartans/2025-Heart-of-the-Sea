package frc.robot.subsystems.scoring.endeffector;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.tunable.TunableNumber;
import frc.robot.subsystems.scoring.constants.ScoringConstants;
import frc.robot.subsystems.scoring.constants.ScoringPIDs;

import java.util.Optional;
import java.util.function.Supplier;

import static frc.robot.subsystems.scoring.constants.ScoringConstants.EndEffectorConstants.*;

public class ConcreteEndEffectorSubsystem extends AbstractEndEffectorSubsystem {
    private final SparkFlex intakeMotor;
    private final SparkFlex wristMotor;
    private final Supplier<Double> wristAbsoluteEncoder;
    private final LaserCan laserCAN;

    private final ProfiledPIDController wristPID;
    private double encoderOffset;
    private final static double DEFAULT_ENCODER_OFFSET = -.29;


    // given is frac 0.82, measured frac 0.556 => need to sub frac 0.264 from measurement
    // sub 0.29663 from measurement

    public ConcreteEndEffectorSubsystem() {
        intakeMotor = new SparkFlex(
            ScoringConstants.IDs.IntakeMotorID,
            SparkLowLevel.MotorType.kBrushless
        );
        wristMotor = new SparkFlex(
            ScoringConstants.IDs.WristMotorID,
            SparkLowLevel.MotorType.kBrushless
        );
        wristAbsoluteEncoder = wristMotor.getAbsoluteEncoder()::getPosition;
        intakeMotor.configure(
            new SparkFlexConfig()
                .apply(
                    new SoftLimitConfig()
                        .forwardSoftLimit(
                            IntakeForwardSoftLimit
                        )
                        .reverseSoftLimit(
                            IntakeReverseSoftLimit
                        )
                ).smartCurrentLimit(IntakeCurrentLimit),
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters
        );
        wristMotor.configure(
            getWristMotorConfig(),
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters
        );
        encoderOffset = DEFAULT_ENCODER_OFFSET;

        wristPID = new ProfiledPIDController(
            ScoringPIDs.wristKp.get(),
            ScoringPIDs.wristKi.get(),
            ScoringPIDs.wristKd.get(),
            new TrapezoidProfile.Constraints(
                ScoringPIDs.wristVelocity.get(),
                ScoringPIDs.wristAcceleration.get()
            )
        );

        ScoringPIDs.wristKp.onChange(wristPID::setP);
        ScoringPIDs.wristKi.onChange(wristPID::setI);
        ScoringPIDs.wristKd.onChange(wristPID::setD);
        TunableNumber.onAnyChange(
            (values) -> wristPID.setConstraints(new TrapezoidProfile.Constraints(values[0], values[1])),
            ScoringPIDs.wristVelocity,
            ScoringPIDs.wristAcceleration
        );

        // DOWN = 0.18 UP = 0.07

        laserCAN = new LaserCan(ScoringConstants.IDs.LaserCANID);
        try {
            laserCAN.setRangingMode(LaserCan.RangingMode.SHORT);
            laserCAN.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
            laserCAN.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
        } catch (ConfigurationFailedException e) {
            System.out.println("Configuration failed! " + e);
        }
        hasCoral = new Trigger(this::hasCoral);
        hasCoral.debounce(3);
        wristMotor.configure(
            new SparkFlexConfig().idleMode(SparkBaseConfig.IdleMode.kCoast),
            SparkBase.ResetMode.kNoResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters
        );
    }

    private static SparkBaseConfig getWristMotorConfig() {
        return new SparkFlexConfig()
            .apply(
                new SoftLimitConfig()
                    .forwardSoftLimit(
                        WristForwardSoftLimit
                    )
                    .reverseSoftLimit(
                        WristReverseSoftLimit
                    )
            ).smartCurrentLimit(WristCurrentLimit);
    }

    @Override
    public Rotation2d getCurrentRotation() {
        return PositionToRotation.convert(
            ((-(wristAbsoluteEncoder.get() - encoderOffset)) % 1 + 1) % 1);
    }

    @Override
    public boolean hasCoral() {
        return Optional.ofNullable(laserCAN.getMeasurement()).map(measurement ->
            measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT && measurement.distance_mm <= 50
        ).orElse(false);
    }

    public double getMeasurement() {
        var measurement = laserCAN.getMeasurement();
        if (measurement == null) {
            return Integer.MAX_VALUE;
        }
        if (measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
            return measurement.distance_mm;
        } else {
            return Integer.MAX_VALUE;
        }
    }

    @Override
    protected void periodic(double targetWristRotationFraction, double intakeSpeed) {
        double currentWristPosition = getCurrentMotorPosition();
        double targetWristPosition = RotationFractionToMotorPosition.convert(targetWristRotationFraction);

        double wristPIDOutput = -wristPID.calculate(currentWristPosition, targetWristPosition);

        SmartDashboard.putNumber("Wrist raw position", wristAbsoluteEncoder.get());
        SmartDashboard.putNumber("LC Measurement", getMeasurement());

        SmartDashboard.putNumber("Wrist Relative Measurement", relativeEncoderMeasurementToAbsoluteMeasurement.convert(wristAbsoluteEncoder.get()) + PositionToRotation.convertBackwards(RotationStartingPosition));
        wristMotor.setVoltage(wristPIDOutput);
        intakeMotor.set(intakeSpeed);
    }

    @Override
    public void setWristMotorIdleMode(SparkBaseConfig.IdleMode mode) {
        wristMotor.configure(getWristMotorConfig().idleMode(mode), SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    }

    private final Debouncer physicalStopDebouncer = new Debouncer(0.06);

    @Override
    public boolean isWristPhysicallyStopped() {
        boolean stopped = Math.abs(wristMotor.getOutputCurrent()) > 25
            && Math.abs(wristMotor.getExternalEncoder().getVelocity()) <= 0.2;
        return physicalStopDebouncer.calculate(stopped);
    }

    @Override
    public void resetCurrentWristRotationFractionTo(double wristRotationFraction) {
        double currentPosition = PositionToRotation.convertBackwards(getCurrentRotation());
        double expectedPosition = PositionToRotation.convertBackwards(ProportionToRotation.convert(wristRotationFraction));
        double positionError = currentPosition - expectedPosition;

        encoderOffset += positionError;
        setTargetWristRotationFraction(wristRotationFraction);
    }
}
