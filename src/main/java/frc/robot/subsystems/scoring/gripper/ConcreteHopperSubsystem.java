package frc.robot.subsystems.scoring.gripper;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.scoring.ScoringSuperstructureState;
import frc.robot.subsystems.scoring.constants.ScoringConstants;
import frc.robot.subsystems.scoring.constants.ScoringPIDs;

public class ConcreteHopperSubsystem extends HopperSubsystem {
    private final SparkFlex intakeMotor, wristMotor;
    private final DutyCycleEncoder wristEncoder;

    private final LaserCan laserCAN;

    private final ProfiledPIDController wristPID;

    private ScoringSuperstructureState scoringState = ScoringSuperstructureState.IDLE;
    private final Trigger hasCoral;

    private boolean isStateFinished = false;

    public ConcreteHopperSubsystem() {
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
                ScoringConstants.WristConstants.WRIST_ABSOLUTE_DOWN_POSITION
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
    public void setHopper(ScoringSuperstructureState state) {
        this.scoringState = state;
        isStateFinished = false;
        intakeMotor.set(state.intakeSpeed);
        wristPID.setGoal(state.getWristAbsolutePosition());
    }

    @Override
    public void periodic() {
        updateConstants();
        runHopper();
    }

    @Override
    public void runHopper() {
        double wristPIDOutput = wristPID.calculate(
                wristEncoder.get(),
                wristPID.getGoal().position
        );
//        uncomment when down and up positions are set
//        wristMotor.set(wristPIDOutput);
        if (atState()) {
            LaserCan.Measurement measurement = laserCAN.getMeasurement();
            if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
                if (scoringState.intakeUntilSeen) {
                    if (hasCoral.getAsBoolean()) {
                        intakeMotor.set(0);
                        isStateFinished = true;
                    }
                } else if (scoringState.outtakeUntilSeen) {
                    if (hasCoral.getAsBoolean()) {
                        intakeMotor.set(0);
                        isStateFinished = true;
                    }
                } else {
                    isStateFinished = true;
                }
            }
        }
    }

    private void updateConstants() {
        wristPID.setPID(
                ScoringPIDs.wristKp.get(),
                ScoringPIDs.wristKi.get(),
                ScoringPIDs.wristKd.get()
        );
        wristPID.setConstraints(
                new TrapezoidProfile.Constraints(
                        ScoringPIDs.wristVelocity.get(),
                        ScoringPIDs.wristAcceleration.get()
                )
        );
    }

    @Override
    protected boolean atState() {
        return MathUtil.isNear(
                getTargetPosition(),
                getCurrentPosition(),
                ScoringConstants.WristConstants.WRIST_TOLERANCE
        );
    }

    @Override
    public double getCurrentPosition() {
        return wristEncoder.get();
    }

    @Override
    public double getTargetPosition() {
        return scoringState.getWristAbsolutePosition();
    }

    public boolean hasCoral() {
        LaserCan.Measurement measurement = laserCAN.getMeasurement();
        if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
            if (scoringState.intakeUntilSeen) {
                return measurement.distance_mm <= 20;
            }
        }
        return false;
    }

    public boolean isStateFinished() {
        return isStateFinished;
    }
}
