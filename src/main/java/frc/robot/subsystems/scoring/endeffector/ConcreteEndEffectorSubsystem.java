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
import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.scoring.ScoringSuperstructureState;
import frc.robot.subsystems.scoring.constants.ScoringConstants;
import frc.robot.subsystems.scoring.constants.ScoringConstants.HopperConstants;
import frc.robot.subsystems.scoring.constants.ScoringPIDs;

import java.util.Optional;

public class ConcreteEndEffectorSubsystem extends AbstractEndEffectorSubsystem {
    private final SparkFlex intakeMotor;
    private final SparkFlex wristMotor;
    private final DutyCycleEncoder wristEncoder;

    private final LaserCan laserCAN;

    private final ProfiledPIDController wristPID;

    private boolean isStateFinished = false;

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
                            HopperConstants.IntakeForwardSoftLimit
                        )
                        .reverseSoftLimit(
                            HopperConstants.IntakeReverseSoftLimit
                        )
                ).smartCurrentLimit(HopperConstants.IntakeCurrentLimit),
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters
        );
        wristMotor.configure(
            new SparkFlexConfig()
                .apply(
                    new SoftLimitConfig()
                        .forwardSoftLimit(
                            HopperConstants.WristForwardSoftLimit
                        )
                        .reverseSoftLimit(
                            HopperConstants.WristReverseSoftLimit
                        )
                ).smartCurrentLimit(HopperConstants.WristCurrentLimit),
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
        //TODO: is two seconds really necessary?
        //having the debounce on for too long could mess with our cycle time
        hasCoral.debounce(2);
        wristMotor.configure(
            new SparkFlexConfig().idleMode(SparkBaseConfig.IdleMode.kBrake),
            SparkBase.ResetMode.kNoResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters
        );
    }

    @Override
    public Rotation2d getCurrentRotation() {
        return HopperConstants.PositionToRotation.convert(wristEncoder.get());
    }

    @Override
    public double getIntakeSpeed() {
        return intakeMotor.get();
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
    public boolean isAtTarget() {
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
        if (!manualControlEnabled) {
            intakeMotor.set(0);
        }
        wristPID.setGoal(state.getWristAbsolutePosition());
    }

    @Override
    public void periodic() {
        if (!manualControlEnabled) {
            if (isAtTarget())
                runHopper();
            else runHopperPosition();
        } else {
            wristMotor.set(wristPID.calculate(
                    wristMotor.getEncoder().getPosition(),
                    HopperConstants.EXTENDED_POSITION/2
            ));
        }
        System.out.println(wristPID.getP() + ", " + wristPID.getI() + ", " + wristPID.getD());

        SmartDashboard.putNumber("Wrist Position", wristMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Wrist Setpoint", HopperConstants.EXTENDED_POSITION/2);
    }

    @Override
    protected void runHopperPosition() {
        if (!manualControlEnabled) {
            double wristPIDOutput = wristPID.calculate(
                wristEncoder.get(),
                wristPID.getGoal().position
            );
        }
//        TODO: uncomment when down and up positions are set
//        wristMotor.set(wristPIDOutput);
    }

    @Override
    public void runHopper() {
        if (!manualControlEnabled) {
            runHopperPosition();
            if (SubsystemManager.getInstance().getScoringSuperstructure().isAtPosition() && !isStateFinished) {
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
}
