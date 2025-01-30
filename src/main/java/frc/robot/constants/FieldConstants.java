// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * Contains various field dimensions and useful reference points. All units are in meters and poses
 * have a blue alliance origin.
 */
public class FieldConstants {
    public static final double fieldLength = Units.inchesToMeters(690.876);
    public static final double fieldWidth = Units.inchesToMeters(317);

    /**
     * Measured from the inside of the starting line.
     */
    public static final double startingLineX =
            Units.inchesToMeters(299.438);

    public static class Processor {
        public static final Pose2d centerFace =
                new Pose2d(Units.inchesToMeters(235.726), 0, Rotation2d.fromDegrees(90));
    }

    public static class Barge {
        public static final Translation2d farCage =
                new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(286.779));
        public static final Translation2d middleCage =
                new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(242.855));
        public static final Translation2d closeCage =
                new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(199.947));

        /**
         * Measured from floor to bottom of cage
         */
        public static final double deepHeight = Units.inchesToMeters(3.125);
        /**
         * Measured from floor to bottom of cage
         */
        public static final double shallowHeight = Units.inchesToMeters(30.125);
    }

    public static class CoralStation {
        public static final Pose2d leftCenterFace =
                new Pose2d(
                        Units.inchesToMeters(33.526),
                        Units.inchesToMeters(291.176),
                        Rotation2d.fromDegrees(90 - 144.011));
        public static final Pose2d rightCenterFace =
                new Pose2d(
                        Units.inchesToMeters(33.526),
                        Units.inchesToMeters(25.824),
                        Rotation2d.fromDegrees(144.011 - 90));
    }

    public static class Reef {
        public static final Translation2d center =
                new Translation2d(Units.inchesToMeters(176.746), Units.inchesToMeters(158.501));
        /**
         * Side of the reef to the inside of the reef zone line
         */
        public static final double faceToZoneLine =
                Units.inchesToMeters(12);
        /**
         * Starting facing the driver station in clockwise order
         */
        public static final Pose2d[] centerFaces = {
                new Pose2d(Units.inchesToMeters(144.003), Units.inchesToMeters(158.500), Rotation2d.fromDegrees(180)),
                new Pose2d(Units.inchesToMeters(160.373), Units.inchesToMeters(186.857), Rotation2d.fromDegrees(120)),
                new Pose2d(Units.inchesToMeters(193.116), Units.inchesToMeters(186.858), Rotation2d.fromDegrees(60)),
                new Pose2d(Units.inchesToMeters(209.489), Units.inchesToMeters(158.502), Rotation2d.fromDegrees(0)),
                new Pose2d(Units.inchesToMeters(193.118), Units.inchesToMeters(130.145), Rotation2d.fromDegrees(-60)),
                new Pose2d(Units.inchesToMeters(160.375), Units.inchesToMeters(130.144), Rotation2d.fromDegrees(-120))
        };
        /**
         * Starting at the right branch facing the driver station going clockwise
         */
        public static final List<Map<ReefHeight, Pose3d>> branchPositions = new ArrayList<>();

        @SuppressWarnings("unused")
        private static List<Map<ReefHeight, Pose3d>> initBranchPositions() {
            List<Map<ReefHeight, Pose3d>> branchPositions = new ArrayList<>();
            // Initialize branch positions
            for (int face = 0; face < 6; face++) {
                Map<ReefHeight, Pose3d> fillRight = new HashMap<>();
                Map<ReefHeight, Pose3d> fillLeft = new HashMap<>();
                for (var level : ReefHeight.values()) {
                    Pose2d poseDirection = new Pose2d(center, Rotation2d.fromDegrees(180 - (60 * face)));
                    double adjustX = Units.inchesToMeters(30.738);
                    double adjustY = Units.inchesToMeters(6.469);

                    fillRight.put(
                            level,
                            new Pose3d(
                                    new Translation3d(
                                            poseDirection
                                                    .transformBy(new Transform2d(adjustX, adjustY, new Rotation2d()))
                                                    .getX(),
                                            poseDirection
                                                    .transformBy(new Transform2d(adjustX, adjustY, new Rotation2d()))
                                                    .getY(),
                                            level.height),
                                    new Rotation3d(
                                            0,
                                            Units.degreesToRadians(level.pitch),
                                            poseDirection.getRotation().getRadians())));
                    fillLeft.put(
                            level,
                            new Pose3d(
                                    new Translation3d(
                                            poseDirection
                                                    .transformBy(new Transform2d(adjustX, -adjustY, new Rotation2d()))
                                                    .getX(),
                                            poseDirection
                                                    .transformBy(new Transform2d(adjustX, -adjustY, new Rotation2d()))
                                                    .getY(),
                                            level.height),
                                    new Rotation3d(
                                            0,
                                            Units.degreesToRadians(level.pitch),
                                            poseDirection.getRotation().getRadians())));
                }
                branchPositions.add((face * 2) + 1, fillRight);
                branchPositions.add((face * 2) + 2, fillLeft);
            }
            return branchPositions;
        }
    }


    /**
     * The three coral + algae spots on each side near the driver stations
     */
    public static class StagingPositions {
        public static final Pose2d leftStagingPosition =
                new Pose2d(Units.inchesToMeters(48), Units.inchesToMeters(230.5), new Rotation2d());
        public static final Pose2d middleStagingPostion =
                new Pose2d(Units.inchesToMeters(48), Units.inchesToMeters(158.5), new Rotation2d());
        public static final Pose2d rightStagingPosition =
                new Pose2d(Units.inchesToMeters(48), Units.inchesToMeters(86.5), new Rotation2d());
    }

    public enum ReefHeight {
        Level4(Units.inchesToMeters(72), -90),
        Level3(Units.inchesToMeters(47.625), -35),
        Level2(Units.inchesToMeters(31.875), -35),
        Level1(Units.inchesToMeters(18), 0);

        ReefHeight(double height, double pitch) {
            this.height = height;
            this.pitch = pitch;
        }

        public final double height;
        /**
         * <b>Units:</b> degrees
         */
        public final double pitch;
    }

    public enum AutonStartingPositions {
        RIGHT_EDGE(new Pose2d(0 + Units.inchesToMeters(RobotConstants.ROBOT_LENGTH / 2), Units.inchesToMeters(49.875 + RobotConstants.ROBOT_WIDTH / 2), Rotation2d.kZero)),
        LEFT_EDGE(new Pose2d(0 + Units.inchesToMeters(RobotConstants.ROBOT_LENGTH / 2), Units.inchesToMeters(Units.metersToInches(fieldWidth) - (49.875 + RobotConstants.ROBOT_WIDTH / 2)), Rotation2d.kZero));

        AutonStartingPositions(Pose2d pose) {
            this.Pose = pose;
        }

        public final Pose2d Pose;
    }

    static Transform2d fromReef = new Transform2d(Units.inchesToMeters(25), 0, Rotation2d.k180deg);
    static Transform2d fromProcessor = new Transform2d(Units.inchesToMeters(25), 0, Rotation2d.k180deg);
    static Transform2d fromCoralStation = new Transform2d(Units.inchesToMeters(25), 0, Rotation2d.k180deg);
    static Transform2d fromBarge = new Transform2d(Units.inchesToMeters(-15), 0, Rotation2d.kZero);

    public enum TargetPositions {
        REEF_0(FieldConstants.Reef.centerFaces[0].transformBy((FieldConstants.fromReef))),
        REEF_1(FieldConstants.Reef.centerFaces[1].transformBy((FieldConstants.fromReef))),
        REEF_2(FieldConstants.Reef.centerFaces[2].transformBy((FieldConstants.fromReef))),
        REEF_3(FieldConstants.Reef.centerFaces[3].transformBy((FieldConstants.fromReef))),
        REEF_4(FieldConstants.Reef.centerFaces[4].transformBy((FieldConstants.fromReef))),
        REEF_5(FieldConstants.Reef.centerFaces[5].transformBy((FieldConstants.fromReef))),

        PROCESSOR(FieldConstants.Processor.centerFace.transformBy(FieldConstants.fromProcessor)),

        CORALSTATION_LEFT(FieldConstants.CoralStation.leftCenterFace.transformBy(FieldConstants.fromCoralStation)),
        CORALSTATION_RIGHT(FieldConstants.CoralStation.rightCenterFace.transformBy(FieldConstants.fromCoralStation)),

        BARGE_FARCAGE(new Pose2d(FieldConstants.Barge.farCage, Rotation2d.kZero).transformBy(fromBarge)),
        BARGE_MIDDLECAGE(new Pose2d(FieldConstants.Barge.middleCage, Rotation2d.kZero).transformBy(fromBarge)),
        BARGE_CLOSECAGE(new Pose2d(FieldConstants.Barge.closeCage, Rotation2d.kZero).transformBy(fromBarge));

        TargetPositions(Pose2d pose) {
            this.Pose = pose;
        }

        public final Pose2d Pose;
    }
}