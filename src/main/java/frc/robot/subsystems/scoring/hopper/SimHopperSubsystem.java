package frc.robot.subsystems.scoring.hopper;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.subsystems.scoring.ScoringSuperstructureState;
import frc.robot.subsystems.scoring.constants.ScoringConstants;
import frc.robot.subsystems.scoring.constants.ScoringPIDs;

public class SimHopperSubsystem extends HopperSubsystem {
    private final ProfiledPIDController pivotPID;
    private final SingleJointedArmSim pivotSim;

    private ScoringSuperstructureState state = ScoringSuperstructureState.IDLE;
    private double intakeSpeed;

    public SimHopperSubsystem() {
        intakeSpeed = state.intakeSpeed;
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
                ScoringSuperstructureState.L2_ALGAE.getWristSimRotation().getRadians(),
                ScoringSuperstructureState.IDLE.getWristSimRotation().getRadians(),
                true,
                ScoringSuperstructureState.IDLE.getWristSimRotation().getRadians()
        );
    }

    @Override
    public double getCurrentPosition() {
        return pivotSim.getAngleRads();
    }

    @Override
    public Rotation2d getCurrentRotation() {
        return Rotation2d.fromRadians(getCurrentPosition());
    }

    @Override
    public double getTargetPosition() {
        return state.getWristSimRotation().getRadians();
    }

    @Override
    public Rotation2d getTargetRotation() {
        return state.getWristSimRotation();
    }

    @Override
    protected boolean isHopperAtPositionState() {
        return MathUtil.isNear(
                ScoringSuperstructureState.getWristSimPosition(getTargetRotation()),
                ScoringSuperstructureState.getWristSimPosition(getCurrentRotation()),
                ScoringConstants.HopperConstants.WRIST_TOLERANCE
        );
    }

    @Override
    public boolean isHopperStateFinished() {
        return isHopperAtPositionState();
    }

    @Override
    public void setHopper(ScoringSuperstructureState state) {
        this.state = state;
        intakeSpeed = state.intakeSpeed;
        pivotPID.setGoal(ScoringSuperstructureState.getWristSimPosition(state.getWristSimRotation()));
    }

    @Override
    public void periodic() {
        updatePIDs();
    }

    @Override
    protected void runHopperPosition() {
        pivotSim.update(0.020);
        pivotSim.setInputVoltage(
                pivotPID.calculate(Rotation2d.fromRadians(pivotSim.getAngleRads()).getRotations())
        );
    }

    @Override
    public void runHopper() {
        runHopperPosition();
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
