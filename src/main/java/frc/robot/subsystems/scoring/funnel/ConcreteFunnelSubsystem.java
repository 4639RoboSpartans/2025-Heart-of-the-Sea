package frc.robot.subsystems.scoring.funnel;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.subsystems.scoring.ScoringSuperstructureState;
import frc.robot.subsystems.scoring.constants.ScoringConstants;
import frc.robot.subsystems.scoring.constants.ScoringConstants.EndEffectorConstants;
import frc.robot.subsystems.scoring.constants.ScoringConstants.FunnelConstants;

public class ConcreteFunnelSubsystem extends AbstractFunnelSubsystem{
    public final SparkFlex pivotMotor;

    private boolean isStateFinished = false;

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
    public void periodic(){
        
    }
    
    @Override
    public boolean isFunnelStateFinished() {
        return true;
    }

    @Override
    public void setFunnel(ScoringSuperstructureState state) {
        this.state = state;
    }

    @Override
    protected void runFunnelPosition() {}
}
