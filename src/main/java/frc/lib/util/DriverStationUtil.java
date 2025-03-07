package frc.lib.util;

import edu.wpi.first.wpilibj.DriverStation;

public class DriverStationUtil {
    //in simulation, change this before starting the simulation as this is what choreo reads as whether to flip alliance.
    private static boolean defaultBlue = false;

    public static DriverStation.Alliance getAlliance() {
        return DriverStation.getAlliance().orElse(
                defaultBlue
                ? DriverStation.Alliance.Blue
                : DriverStation.Alliance.Red
        );
    }
}
