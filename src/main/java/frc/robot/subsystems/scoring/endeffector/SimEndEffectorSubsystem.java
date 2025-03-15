package frc.robot.subsystems.scoring.endeffector;

import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.oi.OI;
import frc.lib.tunable.TunableNumber;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.scoring.ScoringSuperstructureAction;
import frc.robot.subsystems.scoring.constants.ScoringPIDs;

import static frc.robot.subsystems.scoring.constants.ScoringConstants.EndEffectorConstants.*;

public class SimEndEffectorSubsystem extends AbstractEndEffectorSubsystem {
    private double intakeSpeed = 0;

    private final ProfiledPIDController wristPID;
    private final SingleJointedArmSim pivotSim;

    public SimEndEffectorSubsystem() {
        wristPID = new ProfiledPIDController(0, 0, 0, null);
        ScoringPIDs.wristKp.onChange(wristPID::setP);
        ScoringPIDs.wristKi.onChange(wristPID::setI);
        ScoringPIDs.wristKd.onChange(wristPID::setD);
        TunableNumber.onAnyChange(
            (values) -> wristPID.setConstraints(new TrapezoidProfile.Constraints(values[0], values[1])),
            ScoringPIDs.wristVelocity,
            ScoringPIDs.wristAcceleration
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
            ProportionToRotation.convert(1.).getRadians(),
            ProportionToRotation.convert(0.).getRadians(),
            false,
            ProportionToRotation.convert(0.).getRadians()
        );
    }

    @Override
    public Rotation2d getCurrentRotation() {
        return Rotation2d.fromRadians(pivotSim.getAngleRads());
    }

    @Override
    public boolean hasCoral() {
        var currentAction = SubsystemManager.getInstance().getScoringSuperstructure().getCurrentAction();
        return currentAction.endOnGamePieceSeen || currentAction.endOnGamePieceNotSeen
        ? intakeSpeed != 0
        : true;
    }

    @Override
    protected void periodic(double targetWristRotationFraction, double intakeSpeed) {
        this.intakeSpeed = intakeSpeed;
        pivotSim.update(0.020);

        double currentWristPosition = getCurrentMotorPosition();
        double targetWristPosition = RotationFractionToMotorPosition.convert(targetWristRotationFraction);

        double wristPIDOutput = -wristPID.calculate(currentWristPosition, targetWristPosition);

        pivotSim.setInputVoltage(wristPIDOutput);
    }

    @Override
    public void setWristMotorIdleMode(SparkBaseConfig.IdleMode mode) {

    }

    @Override
    public boolean isWristPhysicallyStopped() {
        return pivotSim.hasHitLowerLimit() || pivotSim.hasHitUpperLimit()
            // For testing:
            || OI.getInstance().operatorController().XBOX_START_BUTTON.getAsBoolean();
    }

    @Override
    public void resetCurrentWristRotationFractionTo(double wristRotationFraction) {
        pivotSim.setState(
            ProportionToRotation.convert(wristRotationFraction).getRadians(),
            pivotSim.getVelocityRadPerSec()
        );
        setTargetWristRotationFraction(wristRotationFraction);
        wristPID.reset(PositionToRotation.convertBackwards(ProportionToRotation.convert(wristRotationFraction)));
    }
}