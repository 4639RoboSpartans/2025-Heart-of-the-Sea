package frc.robot.subsystems.scoring.hopper;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
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

import java.util.Optional;

public class ConcreteHopperSubsystem extends HopperSubsystem {
    private final SparkFlex intakeMotor, wristMotor;
    private final DutyCycleEncoder wristEncoder;

    private final LaserCan laserCAN;

    private final ProfiledPIDController wristPID;

    private ScoringSuperstructureState state = ScoringSuperstructureState.IDLE;

    private boolean isStateFinished = false;

    private final ScoringSuperstructure scoringSuperstructure;

    public ConcreteHopperSubsystem() {
        this.scoringSuperstructure = ScoringSuperstructure.getInstance();
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
                            40
                        )
                        .reverseSoftLimit(
                            40
                        )
                ),
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters
        );
        wristMotor.configure(
            new SparkFlexConfig()
                .apply(
                    new SoftLimitConfig()
                        .forwardSoftLimit(
                            30
                        )
                        .reverseSoftLimit(
                            30
                        )
                ),
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters
        );
        wristEncoder = new DutyCycleEncoder(
            ScoringConstants.IDs.WristEncoderID,
            1,
            ScoringConstants.HopperConstants.WRIST_ABSOLUTE_DOWN_POSITION
        );

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
        runHopperPosition();
        if (isHopperAtPosition()) {
            runHopper();
        }
    }

    @Override
    protected void runHopperPosition() {
        @SuppressWarnings("unused")
        double wristPIDOutput = wristPID.calculate(
            wristEncoder.get(),
            wristPID.getGoal().position
        );
        //        for the love of god michael if youre going to comment something out please leave a todo
//        TODO: uncomment when down and up positions are set
//        wristMotor.set(wristPIDOutput);
    }

    @Override
    public void runHopper() {
        runHopperPosition();
        if (scoringSuperstructure.isAtPosition() && !isStateFinished) {
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
