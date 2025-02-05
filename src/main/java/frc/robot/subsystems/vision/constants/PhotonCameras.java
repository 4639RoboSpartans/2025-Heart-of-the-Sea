package frc.robot.subsystems.vision.constants;

import edu.wpi.first.math.geometry.Transform3d;

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