package frc.robot.subsystems.scoring.endeffector;

import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.tunable.TunableNumber;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.scoring.ScoringSuperstructureAction;
import frc.robot.subsystems.scoring.constants.ScoringConstants.EndEffectorConstants;
import frc.robot.subsystems.scoring.constants.ScoringPIDs;

import javax.swing.text.StyleContext;

public class SimEndEffectorSubsystem extends AbstractEndEffectorSubsystem {
    private boolean prevSeenCoral = false;
    private double coralSeenStartTime = 0;
    private final double coralEjectionTime = 2;

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
            EndEffectorConstants.ProportionToRotation.convert(1.).getRadians(),
            EndEffectorConstants.ProportionToRotation.convert(0.).getRadians(),
            false,
            EndEffectorConstants.ProportionToRotation.convert(0.).getRadians()
        );
    }

    @Override
    public Rotation2d getCurrentRotation() {
        return Rotation2d.fromRadians(pivotSim.getAngleRads());
    }

    @Override
    public boolean hasCoral() {
        return SubsystemManager.getInstance().getScoringSuperstructure().getCurrentAction().endOnGamePieceSeen;
    }

    @Override
    protected void periodic(double targetWristRotationFraction, double intakeSpeed) {
        pivotSim.update(0.020);

        double currentWristPosition = getCurrentMotorPosition();
        double targetWristPosition = EndEffectorConstants.RotationFractionToMotorPosition.convert(targetWristRotationFraction);

        double wristPIDOutput = -wristPID.calculate(currentWristPosition, targetWristPosition);

        pivotSim.setInputVoltage(wristPIDOutput);
        SmartDashboard.putNumber("intake speed", intakeSpeed);
    }

    @Override
    public void setWristMotorIdleMode(SparkBaseConfig.IdleMode mode) {

    }
}