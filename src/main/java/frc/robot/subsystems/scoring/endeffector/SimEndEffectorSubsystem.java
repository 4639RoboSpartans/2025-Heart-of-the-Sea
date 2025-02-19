package frc.robot.subsystems.scoring.endeffector;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.oi.OI;
import frc.lib.tunable.TunableNumber;
import frc.robot.subsystems.scoring.constants.ScoringConstants.EndEffectorConstants;
import frc.robot.subsystems.scoring.constants.ScoringPIDs;

public class SimEndEffectorSubsystem extends AbstractEndEffectorSubsystem {
    private final ProfiledPIDController wristPID;
    private final SingleJointedArmSim pivotSim;

    public SimEndEffectorSubsystem() {
        intakeSpeed = 0;

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
        return true;
    }

    @Override
    public void periodic() {
        pivotSim.update(0.020);

        double currentWristPosition = getCurrentPosition();

        double targetWristPosition;
        if (!manualControlEnabled) {
            targetWristPosition = getTargetPosition();
        } else {
            double targetWristProportion = OI.getInstance().operatorController().rightStickY() * 0.5 + 0.5;
            targetWristPosition = EndEffectorConstants.ProportionToPosition.convert(targetWristProportion);
        }

        double wristPIDOutput = -wristPID.calculate(currentWristPosition, targetWristPosition);

        SmartDashboard.putString("Wrist low level info: ",
            "p = " + currentWristPosition + " t = " + targetWristPosition + "o = " + wristPIDOutput
        );

        pivotSim.setInputVoltage(wristPIDOutput);
    }
}