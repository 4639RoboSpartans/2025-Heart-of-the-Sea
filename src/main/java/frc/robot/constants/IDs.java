package frc.robot.constants;

import edu.wpi.first.math.geometry.Transform3d;

public final class IDs {
    private IDs() {}

    public static final int LEFT_CLIMBER_MOTOR = 13;
    public static final int RIGHT_CLIMBER_MOTOR = 14;

    public enum Limelights {
        LIMELIGHT_1("limelight");
        //TODO: add the second limelight

        private final String name;
        Limelights(String name) {
            this.name = name;
        }
        public String getName() {
            return name;
        }
    }

    public enum PhotonCameras {
        //TODO: make an instance here once the PV cam is wired on
        ;
        private final String name;
        private final Transform3d transformFromRobotCenter;

        PhotonCameras(String name, Transform3d transformFromRobotCenter){
            this.name = name;
            this.transformFromRobotCenter = transformFromRobotCenter;
        }

        public String getName(){
            return name;
        }

        public Transform3d getTransformFromRobotCenter(){
            return transformFromRobotCenter;
        }
    }
}
