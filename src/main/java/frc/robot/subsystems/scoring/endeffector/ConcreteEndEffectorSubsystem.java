package frc.robot.subsystems.scoring.endeffector;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.oi.OI;
import frc.lib.tunable.TunableNumber;
import frc.robot.constants.Controls;
import frc.robot.subsystems.scoring.constants.ScoringConstants;
import frc.robot.subsystems.scoring.constants.ScoringConstants.EndEffectorConstants;
import frc.robot.subsystems.scoring.constants.ScoringPIDs;

import java.util.Optional;

public class ConcreteEndEffectorSubsystem extends AbstractEndEffectorSubsystem {
    private final SparkFlex intakeMotor;
    private final SparkFlex wristMotor;
    private final DutyCycleEncoder wristEncoder;

    private final LaserCan laserCAN;

    private final ProfiledPIDController wristPID;

    public ConcreteEndEffectorSubsystem() {
        intakeMotor = new SparkFlex(
            ScoringConstants.IDs.IntakeMotorID,
            SparkLowLevel.MotorType.kBrushless
        );
        wristMotor = new SparkFlex(
            ScoringConstants.IDs.WristMotorID,
            SparkLowLevel.MotorType.kBrushless
        );
        intakeMotor.configure(
            new SparkFlexConfig()
                .apply(
                    new SoftLimitConfig()
                        .forwardSoftLimit(
                            EndEffectorConstants.IntakeForwardSoftLimit
                        )
                        .reverseSoftLimit(
                            EndEffectorConstants.IntakeReverseSoftLimit
                        )
                ).smartCurrentLimit(EndEffectorConstants.IntakeCurrentLimit),
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters
        );
        wristMotor.configure(
            new SparkFlexConfig()
                .apply(
                    new SoftLimitConfig()
                        .forwardSoftLimit(
                            EndEffectorConstants.WristForwardSoftLimit
                        )
                        .reverseSoftLimit(
                            EndEffectorConstants.WristReverseSoftLimit
                        )
                ).smartCurrentLimit(EndEffectorConstants.WristCurrentLimit),
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters
        );
        wristEncoder = new DutyCycleEncoder(
            ScoringConstants.IDs.WristEncoderID,
            1,
            0
        );

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

        laserCAN = new LaserCan(ScoringConstants.IDs.LaserCANID);
        try {
            laserCAN.setRangingMode(LaserCan.RangingMode.SHORT);
            laserCAN.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
            laserCAN.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
        } catch (ConfigurationFailedException e) {
            System.out.println("Configuration failed! " + e);
        }
        hasCoral = new Trigger(this::hasCoral);
        hasCoral.debounce(0.5);
        wristMotor.configure(
            new SparkFlexConfig().idleMode(SparkBaseConfig.IdleMode.kBrake),
            SparkBase.ResetMode.kNoResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters
        );
    }

    @Override
    public Rotation2d getCurrentRotation() {
        return EndEffectorConstants.PositionToRotation.convert(wristEncoder.get());
    }

    @Override
    public boolean hasCoral() {
        return Optional.ofNullable(laserCAN.getMeasurement()).map(measurement ->
            measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT && measurement.distance_mm <= 20
        ).orElse(false);
    }

    @Override
    public boolean isWristAtTarget() {
        return MathUtil.isNear(
            getTargetPosition(),
            getCurrentPosition(),
            EndEffectorConstants.WRIST_TOLERANCE
        );
    }

    @Override
    public void periodic() {
        double currentWristPosition = getCurrentPosition();

        double targetWristPosition, intakeSpeed;
        if (!manualControlEnabled) {
            targetWristPosition = getTargetPosition();
            intakeSpeed = this.intakeSpeed;
        } else {
            double targetWristProportion = OI.getInstance().operatorController().rightStickY() * 0.5 + 0.5;
            targetWristPosition = EndEffectorConstants.ProportionToPosition.convert(targetWristProportion);
            intakeSpeed = Controls.Operator.ManualControlIntake.getAsDouble() * 0.7;
        }

        wristMotor.set(wristPID.calculate(currentWristPosition, targetWristPosition));
        intakeMotor.set(intakeSpeed);

        SmartDashboard.putNumber("Wrist Position", currentWristPosition);
    }
}
