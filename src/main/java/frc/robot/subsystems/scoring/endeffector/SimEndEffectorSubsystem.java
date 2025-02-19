package frc.robot.subsystems.scoring.endeffector;

import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.SubsystemManager;
import frc.lib.oi.OI;
import frc.robot.subsystems.scoring.ScoringSuperstructureState;
import frc.robot.subsystems.scoring.constants.ScoringConstants;
import frc.robot.subsystems.scoring.constants.ScoringPIDs;

public class SimEndEffectorSubsystem extends AbstractEndEffectorSubsystem {
    private static final double secondsUntilIntakeOuttakeEnd = 0.25;

    private final ProfiledPIDController pivotPID;
    private final SingleJointedArmSim pivotSim;

    private double intakeSpeed;

    private double secondsFromIntakeOuttakeStart = 0;

    private boolean isStateFinished = false;

    public SimEndEffectorSubsystem() {
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
            ScoringConstants.EndEffectorConstants.ProportionToRotation.convert(1.).getRadians(),
            ScoringConstants.EndEffectorConstants.ProportionToRotation.convert(0.).getRadians(),
            false,
            ScoringConstants.EndEffectorConstants.ProportionToRotation.convert(0.).getRadians()
        );
    }

    @Override
    public Rotation2d getCurrentRotation() {
        return Rotation2d.fromRadians(pivotSim.getAngleRads());
    }

    @Override
    public double getIntakeSpeed() {
        return intakeSpeed;
    }

    @Override
    public boolean isAtTarget() {
        return MathUtil.isNear(
            ScoringSuperstructureState.getWristSimPosition(getTargetRotation()),
            ScoringSuperstructureState.getWristSimPosition(getCurrentRotation()),
            ScoringConstants.EndEffectorConstants.WRIST_TOLERANCE
        ) && pivotPID.getVelocityError() < 0.01;
    }

    @Override
    public boolean isHopperStateFinished() {
        if (state == ScoringSuperstructureState.IDLE) return isAtTarget();
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
        secondsFromIntakeOuttakeStart = 0;
        pivotPID.setGoal(state.getWristAbsolutePosition());
    }

    @Override
    public void periodic() {
        updatePIDs();
    }

    @Override
    protected void runHopperPosition() {
        pivotSim.update(0.020);
        double output;
        double SIM_VOLTAGE_FUDGE_FACTOR = 0.5;  // Fudge factor to stabilize the simulated hopper
        if (!manualControlEnabled) {
            output = -pivotPID.calculate(getCurrentPosition());
            pivotSim.setInputVoltage(
                output * SIM_VOLTAGE_FUDGE_FACTOR
            );
        } else {
            double setpointProportion = OI.getInstance().operatorController().rightStickY() * 0.5 + 0.5;
            output = -pivotPID.calculate(
                getCurrentPosition(),
                ScoringConstants.EndEffectorConstants
                    .ProportionToPosition
                    .convert(setpointProportion)
            );
            pivotSim.setInputVoltage(output * SIM_VOLTAGE_FUDGE_FACTOR);
        }
        SmartDashboard.putNumber("Wrist Output", output);
        SmartDashboard.putNumber("Wrist Current Position", getCurrentPosition());
        SmartDashboard.putNumber("Wrist Sim Angle", Rotation2d.fromRadians(pivotSim.getAngleRads()).getDegrees());
        SmartDashboard.putNumber("Wrist Target Position", getTargetPosition());
        SmartDashboard.putNumber("Wrist Setpoint Position", pivotPID.getSetpoint().position);
        SmartDashboard.putNumber("Wrist Setpoint Velocity", pivotPID.getSetpoint().velocity);
    }

    @Override
    public void runHopper() {
        runHopperPosition();
        if (SubsystemManager.getInstance().getScoringSuperstructure().isAtPosition() && !isStateFinished) {
            intakeSpeed = state.intakeSpeed;
            secondsFromIntakeOuttakeStart += 0.020;
        }
        if (state.intakeUntilGamePieceSeen) {
            if (hasCoral()) {
                intakeSpeed = 0;
                isStateFinished = true;
            }
        } else if (state.outtakeUntilGamePieceNotSeen) {
            if (hasCoral()) {
                intakeSpeed = 0;
                isStateFinished = true;
            }
        }
    }

    @Override
    public void setWristMotorIdleMode(SparkBaseConfig.IdleMode mode) {

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
