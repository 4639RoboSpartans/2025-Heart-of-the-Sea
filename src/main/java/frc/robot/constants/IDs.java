package frc.robot.constants;

import edu.wpi.first.math.geometry.Transform3d;

public final class IDs {
    private IDs() {}

    public static final int LEFT_CLIMBER_MOTOR = 13;
    public static final int RIGHT_CLIMBER_MOTOR = 14;

    public enum Limelights {
        LIMELIGHT_1("limelight");

        private final String name;
        Limelights(String name) {
            this.name = name;
        }
        public String getName() {
            return name;
        }
    }

    public enum PhotonCameras {
        PLACEHOLDER("", new Transform3d());

        private final String name;
        private final Transform3d transformFromRobotCenter;

        PhotonCameras(String name, Transform3d transformFromRobotCenter){
            this.name = name;
            this.transformFromRobotCenter = transformFromRobotCenter;
        }

        public String getName(){
            return name;
        }

        public Transform3d getTranformFromRobotCenter(){
            return transformFromRobotCenter;
        }
    }
}
