package frc.robot.subsystems.scoring.hopper;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.subsystems.scoring.ScoringSuperstructure;
import frc.robot.subsystems.scoring.ScoringSuperstructureState;
import frc.robot.subsystems.scoring.constants.ScoringConstants;
import frc.robot.subsystems.scoring.constants.ScoringPIDs;

public class SimHopperSubsystem extends HopperSubsystem {
    private static final double secondsUntilIntakeOuttakeEnd = 1;

    private final ProfiledPIDController pivotPID;
    private final SingleJointedArmSim pivotSim;

    private ScoringSuperstructureState state = ScoringSuperstructureState.IDLE;
    private double intakeSpeed;

    private final ScoringSuperstructure scoringSuperstructure;

    private double secondsFromIntakeOuttakeStart = 0;

    private boolean isStateFinished = false;

    public SimHopperSubsystem(ScoringSuperstructure scoringSuperstructure) {
        this.scoringSuperstructure = scoringSuperstructure;
        intakeSpeed = 0;
        pivotPID = new ProfiledPIDController(
                ScoringPIDs.wristKp.get(),
                ScoringPIDs.wristKi.get(),
                ScoringPIDs.wristKd.get(),
                new TrapezoidProfile.Constraints(
                        ScoringPIDs.wristVelocity.get(),
                        ScoringPIDs.wristAcceleration.get()
                )
        );
        pivotSim = new SingleJointedArmSim(
                LinearSystemId.createSingleJointedArmSystem(
                        DCMotor.getNEO(1),
                        SingleJointedArmSim.estimateMOI(0.419, 2.22),
                        25.6
                ),
                DCMotor.getNEO(1),
                25.6,
                0.419,
                ScoringConstants.HopperConstants.EXTENDED_ROTATION.plus(ScoringConstants.HopperConstants.MAX_ROTATION).getRadians(),
                ScoringConstants.HopperConstants.EXTENDED_ROTATION.getRadians(),
                true,
                ScoringConstants.HopperConstants.EXTENDED_ROTATION.getRadians()
        );
    }

    @Override
    public double getCurrentPosition() {
        return ScoringSuperstructureState.getWristSimPosition(getCurrentRotation());
    }

    @Override
    public Rotation2d getCurrentRotation() {
        return Rotation2d.fromRadians(pivotSim.getAngleRads());
    }

    @Override
    public double getTargetPosition() {
        return ScoringSuperstructureState.getWristSimPosition(getTargetRotation());
    }

    @Override
    public Rotation2d getTargetRotation() {
        return state.getWristSimRotation();
    }

    @Override
    public double getIntakeSpeed() {
        return intakeSpeed;
    }

    @Override
    public boolean isHopperAtPosition() {
        return MathUtil.isNear(
                ScoringSuperstructureState.getWristSimPosition(getTargetRotation()),
                ScoringSuperstructureState.getWristSimPosition(getCurrentRotation()),
                ScoringConstants.HopperConstants.WRIST_TOLERANCE
        );
    }

    @Override
    public boolean isHopperStateFinished() {
        return isStateFinished;
    }

    @Override
    public boolean hasCoral() {
        return secondsFromIntakeOuttakeStart >= secondsUntilIntakeOuttakeEnd;
    }

    @Override
    public void setHopper(ScoringSuperstructureState state) {
        this.state = state;
        intakeSpeed = 0;
        isStateFinished = false;
        pivotPID.setGoal(state.getWristAbsolutePosition());
    }

    @Override
    public void periodic() {
        updatePIDs();
    }

    @Override
    protected void runHopperPosition() {
        pivotSim.update(0.020);
        pivotSim.setInputVoltage(
                pivotPID.calculate(getCurrentPosition())
        );
    }

    @Override
    public void runHopper() {
        runHopperPosition();
        if (scoringSuperstructure.isAtPositionState() && !isStateFinished) {
            intakeSpeed = state.intakeSpeed;
            secondsFromIntakeOuttakeStart += 0.020;
        }
        if (state.intakeUntilSeen) {
            if (hasCoral()) {
                intakeSpeed = 0;
                isStateFinished = true;
            }
        } else if (state.outtakeUntilSeen) {
            if (hasCoral()) {
                intakeSpeed = 0;
                isStateFinished = true;
            }
        }
    }

    private void updatePIDs() {
        pivotPID.setPID(
                ScoringPIDs.wristKp.get(),
                ScoringPIDs.wristKi.get(),
                ScoringPIDs.wristKd.get()
        );
        pivotPID.setConstraints(
                new TrapezoidProfile.Constraints(
                        ScoringPIDs.wristVelocity.get(),
                        ScoringPIDs.wristAcceleration.get()
                )
        );
    }
}
