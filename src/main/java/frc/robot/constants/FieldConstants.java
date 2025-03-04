// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.util.AllianceFlipUtil;
import frc.lib.util.PoseUtil;
import frc.robot.subsystems.drive.DriveCommands;

import java.util.HashMap;
import java.util.Map;
import java.util.*;
import java.util.function.Supplier;
import java.util.stream.Stream;

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

        public enum ReefSextant {
            NEAR_CENTER_OF_DRIVER(
                    new Pose2d(Units.inchesToMeters(144.003), Units.inchesToMeters(158.500), Rotation2d.fromDegrees(180))
            ),
            NEAR_LEFT_OF_DRIVER(
                    new Pose2d(Units.inchesToMeters(160.373), Units.inchesToMeters(186.857), Rotation2d.fromDegrees(120))
            ),
            FAR_LEFT_OF_DRIVER(
                    new Pose2d(Units.inchesToMeters(193.116), Units.inchesToMeters(186.858), Rotation2d.fromDegrees(60))
            ),
            FAR_CENTER_OF_DRIVER(
                    new Pose2d(Units.inchesToMeters(209.489), Units.inchesToMeters(158.502), Rotation2d.fromDegrees(0))
            ),
            FAR_RIGHT_OF_DRIVER(
                    new Pose2d(Units.inchesToMeters(193.118), Units.inchesToMeters(130.145), Rotation2d.fromDegrees(-60))
            ),
            NEAR_RIGHT_OF_DRIVER(
                    new Pose2d(Units.inchesToMeters(160.375), Units.inchesToMeters(130.144), Rotation2d.fromDegrees(-120))
            );

            private final Pose2d centerFacePose;
            private final Map<ReefHeight, Pose3d> branchPositionsLeft;
            private final Map<ReefHeight, Pose3d> branchPositionsRight;

            ReefSextant(Pose2d centerFacePose) {
                this.centerFacePose = centerFacePose;

                branchPositionsLeft = new HashMap<>();
                branchPositionsRight = new HashMap<>();
                initBranchPositions();
            }

            public Pose2d centerFacePose() {
                return centerFacePose;
            }

            public Pose3d getBranchPoseLeft(ReefHeight height) {
                return branchPositionsLeft.get(height);
            }

            public Pose3d getBranchPoseRight(ReefHeight height) {
                return branchPositionsRight.get(height);
            }

            private void initBranchPositions() {
                for (ReefHeight level : ReefHeight.values()) {
                    Rotation2d faceRotation = centerFacePose.getRotation();
                    Pose2d poseDirection = new Pose2d(center, faceRotation);
                    double adjustX = Units.inchesToMeters(30.738);
                    double adjustY = Units.inchesToMeters(6.469);

                    Rotation3d rotation = new Rotation3d(
                            0,
                            Units.degreesToRadians(level.pitch),
                            faceRotation.getRadians()
                    );

                    Pose2d poseLeft2D = poseDirection.transformBy(new Transform2d(-adjustX, adjustY, new Rotation2d()));
                    Pose2d poseRight2D = poseDirection.transformBy(new Transform2d(adjustX, adjustY, new Rotation2d()));
                    branchPositionsLeft.put(
                            level,
                            new Pose3d(new Translation3d(poseLeft2D.getX(), poseLeft2D.getY(), level.height), rotation)
                    );
                    branchPositionsRight.put(
                            level,
                            new Pose3d(new Translation3d(poseRight2D.getX(), poseRight2D.getY(), level.height), rotation)
                    );
                }
            }

            public Stream<Pose3d> allBranchPoses() {
                return Stream.concat(
                        branchPositionsLeft.values().stream(),
                        branchPositionsRight.values().stream()
                );
            }

            public static Stream<ReefSextant> all() {
                return Stream.of(values());
            }

            public static Stream<Pose2d> allCenterFacePoses() {
                return all().map(ReefSextant::centerFacePose);
            }

            public static Stream<Pose3d> allBranchPosesAtHeight(ReefHeight height) {
                return all().flatMap(
                        reefSextant -> Stream.of(
                                reefSextant.branchPositionsLeft,
                                reefSextant.branchPositionsRight
                        ).map(j -> j.get(height))
                );
            }

            public static Stream<Pose3d> allBranchPoses(ReefHeight height) {
                return all().flatMap(ReefSextant::allBranchPoses);
            }

            public static ReefSextant closestSextantTo(Pose2d pose) {
                return all().min(
                        Comparator.comparingDouble(
                                a -> a.centerFacePose().getTranslation().getDistance(pose.getTranslation())
                        )
                ).get();
            }

            public static Pose2d closestBranchPoseTo(Pose2d pose) {
                return pose.nearest(allBranchPoses(ReefHeight.Level1).map(Pose3d::toPose2d).toList());
            }
        }

        /**
         * Starting facing the driver station in clockwise order
         */
        @Deprecated
        public static final Pose2d[] centerFaces = {
                new Pose2d(Units.inchesToMeters(144.003), Units.inchesToMeters(158.500), Rotation2d.fromDegrees(180)),
                new Pose2d(Units.inchesToMeters(160.373), Units.inchesToMeters(186.857), Rotation2d.fromDegrees(120)),
                new Pose2d(Units.inchesToMeters(193.116), Units.inchesToMeters(186.858), Rotation2d.fromDegrees(60)),
                new Pose2d(Units.inchesToMeters(209.489), Units.inchesToMeters(158.502), Rotation2d.fromDegrees(0)),
                new Pose2d(Units.inchesToMeters(193.118), Units.inchesToMeters(130.145), Rotation2d.fromDegrees(-60)),
                new Pose2d(Units.inchesToMeters(160.375), Units.inchesToMeters(130.144), Rotation2d.fromDegrees(-120))
        };
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

    static Transform2d fromReef = new Transform2d(Units.inchesToMeters(31), 0, Rotation2d.k180deg);
    static Transform2d fromProcessor = new Transform2d(Units.inchesToMeters(25), 0, Rotation2d.k180deg);
    static Transform2d fromCoralStation = new Transform2d(Units.inchesToMeters(10), 0, Rotation2d.kZero);
    static Transform2d fromBarge = new Transform2d(Units.inchesToMeters(-15), 0, Rotation2d.kZero);

    public enum TargetPositions {
        //TODO: hop on gui and make sure these work
        REEF_AB(
                Reef.centerFaces[0].transformBy((FieldConstants.fromReef)),
                FieldConstants.AprilTagIDHolder.REEF_AB
        ),
        REEF_CD(
                Reef.centerFaces[5].transformBy((FieldConstants.fromReef)),
                FieldConstants.AprilTagIDHolder.REEF_CD
        ),
        REEF_EF(
                Reef.centerFaces[4].transformBy((FieldConstants.fromReef)),
                FieldConstants.AprilTagIDHolder.REEF_EF
        ),
        REEF_GH(
                Reef.centerFaces[3].transformBy((FieldConstants.fromReef)),
                FieldConstants.AprilTagIDHolder.REEF_GH
        ),
        REEF_IJ(
                Reef.centerFaces[2].transformBy((FieldConstants.fromReef)),
                FieldConstants.AprilTagIDHolder.REEF_IJ
        ),
        REEF_KL(
                Reef.centerFaces[1].transformBy((FieldConstants.fromReef)),
                FieldConstants.AprilTagIDHolder.REEF_KL
        ),

        REEF_A(PoseUtil.ReefRelativeLeftOf(REEF_AB.getPose()), FieldConstants.AprilTagIDHolder.REEF_AB, AlignDirection.LEFT),
        REEF_B(PoseUtil.ReefRelativeRightOf(REEF_AB.getPose()), FieldConstants.AprilTagIDHolder.REEF_AB, AlignDirection.RIGHT),
        REEF_C(PoseUtil.ReefRelativeLeftOf(REEF_CD.getPose()), FieldConstants.AprilTagIDHolder.REEF_CD, AlignDirection.LEFT),
        REEF_D(PoseUtil.ReefRelativeRightOf(REEF_CD.getPose()), FieldConstants.AprilTagIDHolder.REEF_CD, AlignDirection.RIGHT),
        REEF_E(PoseUtil.ReefRelativeLeftOf(REEF_EF.getPose()), FieldConstants.AprilTagIDHolder.REEF_EF, AlignDirection.LEFT),
        REEF_F(PoseUtil.ReefRelativeRightOf(REEF_EF.getPose()), FieldConstants.AprilTagIDHolder.REEF_EF, AlignDirection.RIGHT),
        REEF_G(PoseUtil.ReefRelativeLeftOf(REEF_GH.getPose()), FieldConstants.AprilTagIDHolder.REEF_GH, AlignDirection.LEFT),
        REEF_H(PoseUtil.ReefRelativeRightOf(REEF_GH.getPose()), FieldConstants.AprilTagIDHolder.REEF_GH, AlignDirection.RIGHT),
        REEF_I(PoseUtil.ReefRelativeLeftOf(REEF_IJ.getPose()), FieldConstants.AprilTagIDHolder.REEF_IJ, AlignDirection.LEFT),
        REEF_J(PoseUtil.ReefRelativeRightOf(REEF_IJ.getPose()), FieldConstants.AprilTagIDHolder.REEF_IJ, AlignDirection.RIGHT),
        REEF_K(PoseUtil.ReefRelativeLeftOf(REEF_KL.getPose()), FieldConstants.AprilTagIDHolder.REEF_KL, AlignDirection.LEFT),
        REEF_L(PoseUtil.ReefRelativeRightOf(REEF_KL.getPose()), FieldConstants.AprilTagIDHolder.REEF_KL, AlignDirection.RIGHT),

        PROCESSOR(Processor.centerFace.transformBy(FieldConstants.fromProcessor), FieldConstants.AprilTagIDHolder.Processor),

        CORALSTATION_LEFT(CoralStation.leftCenterFace.transformBy(FieldConstants.fromCoralStation), FieldConstants.AprilTagIDHolder.LeftCoralStation),
        CORALSTATION_RIGHT(CoralStation.rightCenterFace.transformBy(FieldConstants.fromCoralStation), FieldConstants.AprilTagIDHolder.RightCoralStation),

        BARGE_FARCAGE(new Pose2d(Barge.farCage, Rotation2d.kZero).transformBy(fromBarge), FieldConstants.AprilTagIDHolder.AllianceBarge),
        BARGE_MIDDLECAGE(new Pose2d(Barge.middleCage, Rotation2d.kZero).transformBy(fromBarge), FieldConstants.AprilTagIDHolder.AllianceBarge),
        BARGE_CLOSECAGE(new Pose2d(Barge.closeCage, Rotation2d.kZero).transformBy(fromBarge), FieldConstants.AprilTagIDHolder.AllianceBarge),

        IDENTITY(new Pose2d(Integer.MAX_VALUE, Integer.MAX_VALUE, Rotation2d.kZero), FieldConstants.AprilTagIDHolder.OpponentBarge);

        TargetPositions(Pose2d pose, AprilTagIDHolder aprilTagIDHolder, AlignDirection direction) {
            this.Pose = pose;
            this.AprilTagIDHolder = aprilTagIDHolder;
            this.direction = direction;
        }

        TargetPositions(Pose2d pose, AprilTagIDHolder aprilTagIDHolder){
            this(pose, aprilTagIDHolder, AlignDirection.NONE);
        }

        private final Pose2d Pose;
        private final AlignDirection direction;

        public FieldConstants.AprilTagIDHolder getAprilTagIDHolder() {
            return AprilTagIDHolder;
        }

        private final AprilTagIDHolder AprilTagIDHolder;

        public Pose2d getPose() {
            return Pose;
        }

        public Pose2d getOpponentAlliancePose() {
            return AllianceFlipUtil.rawAllianceFlipPose(getPose());
        }

        /**
         * Only to be used in autonomous routines for reef R/L align commands
         * @return
         */
        public Supplier<Command> getAlignCommandSupplier() {
            return switch(direction) {
                case LEFT -> () -> DriveCommands.moveToReefLeft(this);
                case RIGHT -> () -> DriveCommands.moveToReefRight(this);
                case NONE -> Commands::none;
            };
        }
    }

    public enum AprilTagIDHolder {
        LeftCoralStation(13, 1),
        RightCoralStation(12, 2),
        Processor(16, 3),
        AllianceBarge(14, 5),
        OpponentBarge(15, 4),
        REEF_AB(18, 7),
        REEF_KL(19, 6),
        REEF_IJ(20, 11),
        REEF_GH(21, 10),
        REEF_EF(22, 9),
        REEF_CD(17, 8);

        public int getRedAllianceID() {
            return redAllianceID;
        }

        public int getBlueAllianceID() {
            return blueAllianceID;
        }

        private final int blueAllianceID;
        private final int redAllianceID;

        AprilTagIDHolder(int blueAllianceID, int redAllianceID) {
            this.blueAllianceID = blueAllianceID;
            this.redAllianceID = redAllianceID;
        }

        public int getAllianceRespectiveID() {
            return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? blueAllianceID : redAllianceID;
        }
    }

    public static double reefForwardsDistance = 0;
    public static double reefSidewaysDistance = 0.125 + Units.inchesToMeters(1.5);

    public enum AlignDirection {
        LEFT,
        RIGHT,
        NONE;
    }
}