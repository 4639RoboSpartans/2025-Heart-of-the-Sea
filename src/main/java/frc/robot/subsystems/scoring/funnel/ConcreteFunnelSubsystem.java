package frc.robot.subsystems.scoring.funnel;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.LinearFilter;
import frc.robot.constants.Controls;
import frc.robot.subsystems.scoring.constants.ScoringConstants;
import frc.robot.subsystems.scoring.constants.ScoringConstants.FunnelConstants;

public class ConcreteFunnelSubsystem extends AbstractFunnelSubsystem{
    public final SparkFlex pivotMotor;

    LinearFilter currents = LinearFilter.movingAverage(10);
    private double avgCurrent;

    private Debouncer debouncer = new Debouncer(1, Debouncer.DebounceType.kBoth);
    public ConcreteFunnelSubsystem(){
        pivotMotor = new SparkFlex(
            ScoringConstants.IDs.FunnelPivotMotorID,
            SparkLowLevel.MotorType.kBrushless
        );
        pivotMotor.configure(
            new SparkFlexConfig()
                .apply(
                    new SoftLimitConfig()
                        .forwardSoftLimit(
                            FunnelConstants.PivotForwardSoftLimit
                        )
                        .reverseSoftLimit(
                            FunnelConstants.PivotReverseSoftLimit
                        )
                ).smartCurrentLimit(FunnelConstants.PivotCurrentLimit),
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters
        );
    }

    @Override
    public double getCurrent(){
        return pivotMotor.getOutputCurrent();
    }

    @Override
    public void periodic(){
        avgCurrent = currents.calculate(getCurrent());

        if (!isManualControlEnabled && isFunnelStateFinished()) {
            double voltage = 10;
            if(!isTargetPositionDown){
                voltage *= -1;
            }
            pivotMotor.setVoltage(voltage);
            if(debouncer.calculate(avgCurrent > 20)){
                pivotMotor.setVoltage(0);
                isDown = isTargetPositionDown;
            }
        }
        else{
            double outputVoltage = Controls.Operator.ManualControlElevator.getAsDouble();
            pivotMotor.setVoltage(outputVoltage);
        }
    }
}
