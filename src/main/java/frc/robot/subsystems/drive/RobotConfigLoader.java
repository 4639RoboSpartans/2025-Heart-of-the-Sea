package frc.robot.subsystems.drive;

import com.pathplanner.lib.config.RobotConfig;

public class RobotConfigLoader {
    private static RobotConfig config;

    public static RobotConfig getOrLoadConfig() {
        if (config == null) {
            try {
                config = RobotConfig.fromGUISettings();
            } catch (Exception e) {
                throw new RuntimeException("Swerve Config Failed", e);
            }
        }
        return config;
    }
}
