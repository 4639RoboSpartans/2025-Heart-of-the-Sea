package frc.robot.subsystems.scoring.elevator;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystemManager.Subsystems;

import static edu.wpi.first.units.Units.*;

public class ElevatorSysID {
    private static final AbstractElevatorSubsystem elevator = Subsystems.scoringSuperstructure().getElevatorSubsystem();

    private static final SysIdRoutine elevatorRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                    Volts.of(4).per(Second),
                    Volts.of(3),
                    Seconds.of(3),
                    (state) -> SignalLogger.writeString("state", state.toString())
            ),
            new SysIdRoutine.Mechanism(
                    elevator::setRawMotorVoltage,
                    null,
                    elevator
            )
    );

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #elevatorRoutine}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public static Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return elevatorRoutine.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #elevatorRoutine}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public static Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return elevatorRoutine.dynamic(direction);
    }
}
