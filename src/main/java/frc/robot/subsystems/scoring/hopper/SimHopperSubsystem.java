package frc.robot.subsystems.scoring.hopper;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.algaeIntake.constants.AlgaeIntakeConstants;
import frc.robot.subsystems.scoring.ScoringSuperstructureState;

public class SimHopperSubsystem extends HopperSubsystem {
    private final ProfiledPIDController pivotPID;
    private final SingleJointedArmSim pivotSim;

    private ScoringSuperstructureState state = ScoringSuperstructureState.IDLE;
    private Rotation2d simAngle;
    private double intakeSpeed;

    public SimHopperSubsystem() {
        simAngle = state.getSimAngle();
        intakeSpeed = state.intakeSpeed;
        pivotPID = new ProfiledPIDController(
                AlgaeIntakePIDs.pivotKp.get(),
                AlgaeIntakePIDs.pivotKi.get(),
                AlgaeIntakePIDs.pivotKd.get(),
                new TrapezoidProfile.Constraints(
                        AlgaeIntakePIDs.pivotVelocity.get(),
                        AlgaeIntakePIDs.pivotAcceleration.get()
                )
        );
        pivotSim = new SingleJointedArmSim(
                LinearSystemId.createSingleJointedArmSystem(
                        DCMotor.getNEO(2),
                        SingleJointedArmSim.estimateMOI(0.6, 30),
                        200
                ),
                DCMotor.getNEO(2),
                200.0,
                0.6,
                AlgaeIntakeState.EXTENDED.getSimAngle().getRadians(),
                AlgaeIntakeState.IDLE.getSimAngle().getRadians(),
                true,
                AlgaeIntakeState.IDLE.getSimAngle().getRadians()
        );
    }

    protected void setState(AlgaeIntakeState state) {
        this.state = state;
        intakeSpeed = state.intakeSpeed;
        pivotPID.setGoal(state.getSimAngle().getRotations());
    }

    public Trigger atRequestedState() {
        return new Trigger(
                () -> MathUtil.isNear(
                        pivotPID.getGoal().position,
                        Rotation2d.fromRadians(pivotSim.getAngleRads()).getRotations(),
                        AlgaeIntakeConstants.PivotConstants.positionTolerance
                )
        );
    }

    @Override
    public void periodic() {
        updatePIDs();
        runPivot();
    }

    private void runPivot() {
        pivotSim.setInputVoltage(
                pivotPID.calculate(Rotation2d.fromRadians(pivotSim.getAngleRads()).getRotations())
        );
    }

    private void updatePIDs() {
        pivotPID.setPID(
                AlgaeIntakePIDs.pivotKp.get(),
                AlgaeIntakePIDs.pivotKi.get(),
                AlgaeIntakePIDs.pivotKd.get()
        );
        pivotPID.setConstraints(
                new TrapezoidProfile.Constraints(
                        AlgaeIntakePIDs.pivotVelocity.get(),
                        AlgaeIntakePIDs.pivotAcceleration.get()
                )
        );
    }
}
