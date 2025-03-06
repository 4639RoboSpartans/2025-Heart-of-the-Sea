package frc.lib.util;

import edu.wpi.first.wpilibj.DriverStation;

public class DriverStationUtil {
    public static DriverStation.Alliance getAlliance() {
        return DriverStation.getAlliance().orElse(DriverStation.Alliance.Red);
    }
}
