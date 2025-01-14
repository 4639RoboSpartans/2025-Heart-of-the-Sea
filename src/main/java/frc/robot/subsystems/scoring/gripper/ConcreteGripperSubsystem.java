package frc.robot.subsystems.scoring.gripper;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.scoring.ScoringSuperstructureState;
import frc.robot.subsystems.scoring.constants.ScoringConstants;
import frc.robot.subsystems.scoring.constants.ScoringPIDs;

public class ConcreteGripperSubsystem extends GripperSubsystem {
    private final SparkFlex intakeMotor, wristMotor;
    private final DutyCycleEncoder wristEncoder;

    private final LaserCan laserCAN;

    private final ProfiledPIDController wristPID;

    public ConcreteGripperSubsystem() {
        intakeMotor = new SparkFlex(
                ScoringConstants.IDs.IntakeMotorID,
                SparkLowLevel.MotorType.kBrushless
        );
        wristMotor = new SparkFlex(
                ScoringConstants.IDs.WristMotorID,
                SparkLowLevel.MotorType.kBrushless
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
    }

    @Override
    public void setGripperState(ScoringSuperstructureState state) {
        intakeMotor.set(state.intakeSpeed);
        wristPID.setGoal(state.getWristAbsolutePosition());
    }

    @Override
    public void runGripper() {
        double wristPIDOutput = wristPID.calculate(
                wristEncoder.get(),
                wristPID.getGoal().position
        );
        wristMotor.set(wristPIDOutput);LaserCan.Measurement measurement = laserCAN.getMeasurement();
        if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
            if (measurement.distance_mm <= 20) {
                intakeMotor.set(0);
            }
        }
    }

    @Override
    protected boolean atState() {
        return MathUtil.isNear(
                wristPID.getGoal().position,
                wristEncoder.get(),
                ScoringConstants.WristConstants.WRIST_TOLERANCE
        );
    }

    @Override
    public double getCurrentPosition() {
        return 0;
    }

    @Override
    public double getTargetPosition() {
        return 0;
    }

    @Override
    public Command quasistatic(SysIdRoutine.Direction direction) {
        return null;
    }

    @Override
    public Command dynamic(SysIdRoutine.Direction direction) {
        return null;
    }
}
