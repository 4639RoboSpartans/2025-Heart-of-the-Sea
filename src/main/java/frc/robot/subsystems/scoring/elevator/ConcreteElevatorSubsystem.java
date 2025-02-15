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

import static frc.robot.subsystems.scoring.constants.ScoringConstants.ElevatorConstants;
import static frc.robot.subsystems.scoring.constants.ScoringConstants.IDs;
import static frc.robot.subsystems.scoring.constants.ScoringPIDs.*;

public class ConcreteElevatorSubsystem extends ElevatorSubsystem {
    private final TalonFX elevatorMotor;
    private final MotionMagicVoltage controlRequest;

    private ScoringSuperstructureState state = ScoringSuperstructureState.IDLE;

    private boolean isStateFinished = false;

    @SuppressWarnings("resource")
    public ConcreteElevatorSubsystem() {
        TalonFX leftElevatorMotor = new TalonFX(IDs.ElevatorLeftID, IDs.ElevatorCANBusName);
        TalonFX rightElevatorMotor = new TalonFX(IDs.ElevatorRightID, IDs.ElevatorCANBusName);
        leftElevatorMotor.setNeutralMode(NeutralModeValue.Brake);
        rightElevatorMotor.setNeutralMode(NeutralModeValue.Brake);
        var leftConfigurator = leftElevatorMotor.getConfigurator();
        var rightConfigurator = rightElevatorMotor.getConfigurator();
        TalonFXConfiguration configuration = new TalonFXConfiguration()
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicAcceleration(elevatorAcceleration.get())
                    .withMotionMagicCruiseVelocity(elevatorVelocity.get())
            )
            .withSlot0(
                new Slot0Configs()
                    .withKP(elevatorKp.get())
                    .withKI(elevatorKi.get())
                    .withKD(elevatorKd.get())
                    .withKA(elevatorKa.get())
                    .withKS(elevatorKs.get())
                    .withKV(elevatorKv.get())
                    .withKG(elevatorKg.get())
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
        rightElevatorMotor.setControl(new Follower(IDs.ElevatorLeftID, true));

        elevatorMotor = leftElevatorMotor;

        controlRequest = new MotionMagicVoltage(leftElevatorMotor.getPosition().getValueAsDouble());
    }

    @Override
    public double getCurrentPosition() {
        return elevatorMotor.getPosition(true).getValueAsDouble();
    }

    @Override
    public Distance getCurrentLength() {
        return ElevatorConstants.PositionToHeight.convert(getCurrentPosition());
    }

    @Override
    public double getTargetPosition() {
        return state.getElevatorAbsolutePosition();
    }

    @Override
    public Distance getTargetLength() {
        return ElevatorConstants.PositionToHeight.convert(getTargetPosition());
    }

    public boolean isElevatorStateFinished() {
        return isStateFinished;
    }

    public boolean isElevatorAtPosition() {
        return MathUtil.isNear(
            controlRequest.Position,
            elevatorMotor.getPosition().getValueAsDouble(),
            ElevatorConstants.ELEVATOR_TOLERANCE
        );
    }

    public void setElevatorState(ScoringSuperstructureState state) {
        this.state = state;
        isStateFinished = false;
        controlRequest.Position = state.getElevatorAbsolutePosition();
    }

    public void runElevator() {
        elevatorMotor.setControl(controlRequest);
        SmartDashboard.putNumber("output", elevatorMotor.getMotorVoltage().getValueAsDouble());
    }

    @Override
    public void periodic() {
        if (isElevatorAtPosition()) {
            isStateFinished = true;
        }
        SmartDashboard.putNumber("Elevator Proportion", ElevatorConstants.ProportionToPosition.convertBackwards(getCurrentPosition()));
        SmartDashboard.putBoolean("At State", isElevatorAtPosition());
        SignalLogger.writeDouble("Elevator Position", elevatorMotor.getPosition().getValueAsDouble());
        SignalLogger.writeDouble("Elevator Velocity", elevatorMotor.getVelocity().getValueAsDouble());
        SignalLogger.writeDouble("Elevator Acceleration", elevatorMotor.getAcceleration().getValueAsDouble());
        SignalLogger.writeDouble("Elevator Voltage", elevatorMotor.getMotorVoltage().getValueAsDouble());
    }

    @Override
    public void setElevatorMotorVoltsSysID(Voltage voltage) {
        elevatorMotor.setControl(new VoltageOut(voltage));
    }
}
