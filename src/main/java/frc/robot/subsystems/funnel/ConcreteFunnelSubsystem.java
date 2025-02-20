package frc.robot.subsystems.funnel;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkFlexConfig;

import frc.robot.subsystems.scoring.ScoringSuperstructureState;
import frc.robot.subsystems.scoring.constants.ScoringConstants;

// TODO: implement this correctly
public class ConcreteFunnelSubsystem extends AbstractFunnelSubsystem{
    public final SparkFlex pivotMotor;

    public ConcreteFunnelSubsystem(){
        pivotMotor = new SparkFlex(
            FunnelConstants.FunnelPivotMotorID,
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
    public void periodic(){
        
    }
}
