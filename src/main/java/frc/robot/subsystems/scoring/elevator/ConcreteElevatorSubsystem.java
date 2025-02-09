package frc.robot.subsystems.scoring.elevator;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.scoring.ScoringSuperstructureState;
import frc.robot.subsystems.scoring.constants.ScoringConstants;
import frc.robot.subsystems.scoring.constants.ScoringPIDs;

public class ConcreteElevatorSubsystem extends ElevatorSubsystem {
    private final TalonFX leftElevatorMotor, rightElevatorMotor;
    private final MotionMagicVoltage controlRequest;

    private ScoringSuperstructureState state = ScoringSuperstructureState.IDLE;


    private boolean isStateFinished = false;

    public ConcreteElevatorSubsystem() {
        leftElevatorMotor = new TalonFX(ScoringConstants.IDs.ElevatorLeftID, ScoringConstants.IDs.ElevatorCANBusName);
        rightElevatorMotor = new TalonFX(ScoringConstants.IDs.ElevatorRightID, ScoringConstants.IDs.ElevatorCANBusName);
        leftElevatorMotor.setNeutralMode(NeutralModeValue.Brake);
        rightElevatorMotor.setNeutralMode(NeutralModeValue.Brake);
        var leftConfigurator = leftElevatorMotor.getConfigurator();
        var rightConfigurator = rightElevatorMotor.getConfigurator();
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
                                .withKA(ScoringPIDs.elevatorKa.get())
                                .withKS(ScoringPIDs.elevatorKs.get())
                                .withKV(ScoringPIDs.elevatorKv.get())
                                .withKG(ScoringPIDs.elevatorKg.get())
                                .withGravityType(GravityTypeValue.Elevator_Static)
                )
                .withCurrentLimits(
                        new CurrentLimitsConfigs()
                                .withStatorCurrentLimit(
                                        30
                                )
                );

        leftConfigurator.apply(configuration);
        rightConfigurator.apply(configuration);
        leftElevatorMotor.setNeutralMode(NeutralModeValue.Brake);
        rightElevatorMotor.setNeutralMode(NeutralModeValue.Brake);
        rightElevatorMotor.setControl(new Follower(ScoringConstants.IDs.ElevatorLeftID, true));
        controlRequest = new MotionMagicVoltage(leftElevatorMotor.getPosition().getValueAsDouble());
    }

    @Override
    public double getCurrentPosition() {
        return leftElevatorMotor.getPosition(true).getValueAsDouble();
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
                leftElevatorMotor.getPosition().getValueAsDouble(),
                ScoringConstants.ElevatorConstants.ELEVATOR_TOLERANCE
        );
    }

    public void setElevatorState(ScoringSuperstructureState state) {
        this.state = state;
        isStateFinished = false;
        controlRequest.Position = state.getElevatorAbsolutePosition();
    }

    public void runElevator() {
        leftElevatorMotor.setControl(controlRequest);
        SmartDashboard.putNumber("output", leftElevatorMotor.getMotorVoltage().getValueAsDouble());
    }

    @Override
    public void periodic() {
        if (isElevatorAtPosition()) {
            isStateFinished = true;
        }
        SmartDashboard.putNumber("Elevator Position", getCurrentPosition());
        SmartDashboard.putNumber("Elevator Proportion", ScoringConstants.ElevatorConstants.ProportionToPosition.convertBackwards(getCurrentPosition()));
        SmartDashboard.putBoolean("At State", isElevatorAtPosition());
        SignalLogger.writeDouble("Elevator Position", leftElevatorMotor.getPosition().getValueAsDouble());
        SignalLogger.writeDouble("Elevator Velocity", leftElevatorMotor.getVelocity().getValueAsDouble());
        SignalLogger.writeDouble("Elevator Acceleration", leftElevatorMotor.getAcceleration().getValueAsDouble());
        SignalLogger.writeDouble("Elevator Volage", leftElevatorMotor.getMotorVoltage().getValueAsDouble());
    }

    @Override
    public void setElevatorMotorVoltsSysID(Voltage voltage) {
        leftElevatorMotor.setControl(new VoltageOut(voltage));
    }
}
