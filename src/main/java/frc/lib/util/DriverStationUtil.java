package frc.lib.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class DriverStationUtil {
    // in simulation, change this before starting the simulation as this is what choreo reads as whether to flip alliance.
    private static final boolean defaultBlue = true;

    public static DriverStation.Alliance getAlliance() {
        return DriverStation.getAlliance().orElseGet(
                () -> defaultBlue? DriverStation.Alliance.Blue : DriverStation.Alliance.Red
        );
    }

    public static boolean hasDSAlliance() {
        return DriverStation.getAlliance().isPresent();
    }

    public static Trigger hasDSAlliance = new Trigger(DriverStationUtil::hasDSAlliance);
}
