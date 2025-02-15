package frc.robot.subsystems.scoring.hopper;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.scoring.ScoringSuperstructure;
import frc.robot.subsystems.scoring.ScoringSuperstructureState;
import frc.robot.subsystems.scoring.constants.ScoringConstants;

public class DummyHopperSubsystem extends HopperSubsystem {
    ScoringSuperstructureState scoringState;

    public DummyHopperSubsystem() {
        super();
        scoringState = ScoringSuperstructureState.IDLE;
    }

    @Override
    public double getCurrentPosition() {
        return scoringState.getWristAbsolutePosition();
    }

    @Override
    public Rotation2d getCurrentRotation() {
        return scoringState.getWristSimRotation();
    }

    @Override
    public double getTargetPosition() {
        return getCurrentPosition();
    }

    @Override
    public Rotation2d getTargetRotation() {
        return getCurrentRotation();
    }

    @Override
    public double getIntakeSpeed() {
        return 0;
    }

    @Override
    public ScoringSuperstructureState getHopperState() {
        return scoringState;
    }

    @Override
    public boolean isHopperAtPosition() {
        return true;
    }

    @Override
    public boolean isHopperStateFinished() {
        return true;
    }

    @Override
    public boolean hasCoral() {
        return false;
    }

    @Override
    public void setHopper(ScoringSuperstructureState state) {
        scoringState = state;
    }

    @Override
    protected void runHopperPosition() {
        //massive low
    }

    @Override
    public void runHopper() {
        //taper fade
    }
}
