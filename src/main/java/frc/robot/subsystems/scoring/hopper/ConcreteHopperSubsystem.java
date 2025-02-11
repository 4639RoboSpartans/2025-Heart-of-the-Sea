package frc.robot.subsystems.scoring.hopper;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.TunableNumber;
import frc.robot.subsystems.scoring.ScoringSuperstructure;
import frc.robot.subsystems.scoring.ScoringSuperstructureState;
import frc.robot.subsystems.scoring.constants.ScoringConstants;
import frc.robot.subsystems.scoring.constants.ScoringPIDs;
import frc.robot.subsystems.scoring.constants.ScoringConstants.HopperConstants;

import java.util.Optional;

public class ConcreteHopperSubsystem extends HopperSubsystem {
    private final SparkFlex intakeMotor;
    private final TalonFX wristMotor;
    private final DutyCycleEncoder wristEncoder;

    private final LaserCan laserCAN;

    private final ProfiledPIDController wristPID;

    private ScoringSuperstructureState state = ScoringSuperstructureState.IDLE;

    private boolean isStateFinished = false;

    public ConcreteHopperSubsystem() {
        intakeMotor = new SparkFlex(
            ScoringConstants.IDs.IntakeMotorID,
            SparkLowLevel.MotorType.kBrushless
        );
        wristMotor = new TalonFX(
                ScoringConstants.IDs.WristMotorID,
                ScoringConstants.IDs.WristCANBusName
        );
        intakeMotor.configure(
            new SparkFlexConfig()
                .apply(
                    new SoftLimitConfig()
                        .forwardSoftLimit(
                            HopperConstants.IntakeForwardSoftLimit
                        )
                        .reverseSoftLimit(
                            HopperConstants.IntakeReverseSoftLimit
                        )
                ).smartCurrentLimit(HopperConstants.IntakeCurrentLimit),
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters
        );
        wristMotor.getConfigurator().apply(
                new SoftwareLimitSwitchConfigs()
                        .withForwardSoftLimitThreshold(HopperConstants.WristForwardSoftLimit)
                        .withReverseSoftLimitThreshold(HopperConstants.WristReverseSoftLimit)
        );
        wristMotor.getConfigurator().apply(
                new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(HopperConstants.WristCurrentLimit)
        );
        wristEncoder = new DutyCycleEncoder(
            ScoringConstants.IDs.WristEncoderID,
            1,
            0
        );

        //TODO: now that we are running a Talon instead of a NEO, probably better to use motion magic
        wristPID = new ProfiledPIDController(0, 0, 0, null);

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
        //TODO: is two seconds really necessary?
        //having the debounce on for too long could mess with our cycle time
        hasCoral.debounce(2);
    }

    @Override
    public double getCurrentPosition() {
        return wristEncoder.get();
    }

    @Override
    public Rotation2d getCurrentRotation() {
        return ScoringSuperstructureState.getWristSimRotation(getCurrentPosition());
    }

    @Override
    public double getTargetPosition() {
        return state.getWristAbsolutePosition();
    }

    @Override
    public Rotation2d getTargetRotation() {
        return ScoringSuperstructureState.getWristSimRotation(getTargetPosition());
    }

    @Override
    public double getIntakeSpeed() {
        return intakeMotor.get();
    }

    @Override
    public ScoringSuperstructureState getHopperState() {
        return state;
    }

    @Override
    public boolean hasCoral() {
        return Optional.ofNullable(laserCAN.getMeasurement()).map(measurement ->
            measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT
                && measurement.distance_mm <= 20
        ).orElse(false);

        /*
        no reason to check intake state at this point
         */
    }

    @Override
    public boolean isHopperAtPosition() {
        return MathUtil.isNear(
            getTargetPosition(),
            getCurrentPosition(),
            ScoringConstants.HopperConstants.WRIST_TOLERANCE
        );
    }

    public boolean isHopperStateFinished() {
        return isStateFinished;
    }

    @Override
    public void setHopper(ScoringSuperstructureState state) {
        this.state = state;
        isStateFinished = false;
        intakeMotor.set(0);
        wristPID.setGoal(state.getWristAbsolutePosition());
    }

    @Override
    public void periodic() {
        if (isHopperAtPosition())
            runHopper();
        else runHopperPosition();
    }

    @Override
    protected void runHopperPosition() {
        @SuppressWarnings("unused")
        double wristPIDOutput = wristPID.calculate(
            wristEncoder.get(),
            wristPID.getGoal().position
        );
//        TODO: uncomment when down and up positions are set
//        wristMotor.set(wristPIDOutput);
    }

    @Override
    public void runHopper() {
        runHopperPosition();
        if (ScoringSuperstructure.getInstance().isAtPosition() && !isStateFinished) {
            intakeMotor.set(state.intakeSpeed);
        }
        LaserCan.Measurement measurement = laserCAN.getMeasurement();
        if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
            if (state.intakeUntilGamePieceSeen) {
                if (hasCoral()) {
                    intakeMotor.set(0);
                    isStateFinished = true;
                }
            } else if (state.outtakeUntilGamePieceNotSeen) {
                if (!hasCoral()) {
                    intakeMotor.set(0);
                    isStateFinished = true;
                }
            }
        }
    }
}
