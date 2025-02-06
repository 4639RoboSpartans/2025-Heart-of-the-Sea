package frc.robot.subsystems.scoring.elevator;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.scoring.ScoringSuperstructureState;
import frc.robot.subsystems.scoring.constants.ScoringConstants;
import frc.robot.subsystems.scoring.constants.ScoringPIDs;

public class ConcreteElevatorSubsystem extends ElevatorSubsystem {
    private final TalonFX leftElevator, rightElevator;
    private final MotionMagicVoltage controlRequest;

    private ScoringSuperstructureState state = ScoringSuperstructureState.IDLE;

    private boolean isStateFinished = false;

    public ConcreteElevatorSubsystem() {
        leftElevator = new TalonFX(ScoringConstants.IDs.ElevatorLeftID, ScoringConstants.IDs.ElevatorCANBusName);
        rightElevator = new TalonFX(ScoringConstants.IDs.ElevatorRightID, ScoringConstants.IDs.ElevatorCANBusName);
        leftElevator.setNeutralMode(NeutralModeValue.Brake);
        rightElevator.setNeutralMode(NeutralModeValue.Brake);
        var leftConfigurator = leftElevator.getConfigurator();
        var rightConfigurator = rightElevator.getConfigurator();
        TalonFXConfiguration configuration = new TalonFXConfiguration()
                .withMotionMagic(
                        new MotionMagicConfigs()
                                .withMotionMagicAcceleration(ScoringPIDs.elevatorAcceleration.get())
                                .withMotionMagicCruiseVelocity(ScoringPIDs.elevatorVelocity.get()))
                .withSlot0(
                        new Slot0Configs()
                                .withKP(ScoringPIDs.elevatorKp.get())
                                .withKI(ScoringPIDs.elevatorKi.get())
                                .withKD(ScoringPIDs.elevatorKd.get())
                );
        leftConfigurator.apply(configuration);
        rightConfigurator.apply(configuration);
        leftElevator.setNeutralMode(NeutralModeValue.Brake);
        rightElevator.setNeutralMode(NeutralModeValue.Brake);
        rightElevator.setControl(new Follower(ScoringConstants.IDs.ElevatorLeftID, true));
        controlRequest = new MotionMagicVoltage(leftElevator.getPosition().getValueAsDouble());
    }

    @Override
    public double getCurrentPosition() {
        return leftElevator.getPosition(true).getValueAsDouble();
    }

    @Override
    public Distance getCurrentLength() {
        return ScoringSuperstructureState.getElevatorSimDistance(getCurrentPosition());
    }

    @Override
    public double getTargetPosition() {
        return state.getElevatorAbsolutePosition();
    }

    @Override
    public Distance getTargetLength() {
        return ScoringSuperstructureState.getElevatorSimDistance(getTargetPosition());
    }

    public boolean isElevatorStateFinished() {
        return isStateFinished;
    }

    public boolean isElevatorAtPosition() {
        return MathUtil.isNear(
                controlRequest.Position,
                leftElevator.getPosition().getValueAsDouble(),
                ScoringConstants.ElevatorConstants.ELEVATOR_TOLERANCE
        );
    }

    public void setElevatorState(ScoringSuperstructureState state) {
        this.state = state;
        isStateFinished = false;
        controlRequest.Position = state.getElevatorAbsolutePosition();
    }

    public void runElevator() {
//        uncomment when down and up positions are set
        // leftElevator.setControl(controlRequest);
        SmartDashboard.putNumber("output", leftElevator.getMotorVoltage().getValueAsDouble());
    }

    @Override
    public void periodic() {
        if (isElevatorAtPosition()) {
            isStateFinished = true;
        }
        SmartDashboard.putNumber("ELevator Position", getCurrentPosition());
        SmartDashboard.putBoolean("At State", isElevatorAtPosition());
    }

    @Override
    public void setElevatorMotorVoltsSysID(Voltage voltage) {
        leftElevator.setControl(new VoltageOut(voltage));
    }
}
