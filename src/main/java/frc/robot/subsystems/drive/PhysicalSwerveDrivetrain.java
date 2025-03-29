package frc.robot.subsystems.drive;

import choreo.trajectory.SwerveSample;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.*;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import edu.wpi.first.math.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.limelight.LimelightHelpers;
import frc.lib.util.DriverStationUtil;
import frc.robot.constants.Controls;
import frc.robot.constants.Limelights;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.drive.constants.DriveConstants;
import frc.robot.subsystems.drive.constants.DrivePIDs;
import frc.robot.subsystems.drive.constants.TunerConstants;
import frc.robot.subsystems.drive.constants.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.subsystems.vision.Vision;

import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;

public class PhysicalSwerveDrivetrain extends AbstractSwerveDrivetrain {
    protected final TunerSwerveDrivetrain drivetrain;

    private boolean didApplyOperatorPerspective = false;

    private final PhoenixPIDController headingController = new PhoenixPIDController(6, 0, 0);
    protected final PIDController
            pathXController = new PIDController(1, 0, 0),
            pathYController = new PIDController(1, 0, 0),
            pathHeadingController = new PIDController(10, 0, 0);
    protected ProfiledPIDController
            pidXController = constructPIDXController();
    protected ProfiledPIDController pidYController = constructPIDYController();

    public static ProfiledPIDController constructPIDYController() {
        return new ProfiledPIDController(DrivePIDs.pidToPoseXkP.get(), 0, 0, new TrapezoidProfile.Constraints(4, 2));
    }

    public static ProfiledPIDController constructPIDXController() {
        return new ProfiledPIDController(DrivePIDs.pidToPoseYkP.get(), 0, 0, new TrapezoidProfile.Constraints(4, 2));
    }

    {
        pathHeadingController.enableContinuousInput(-Math.PI, Math.PI);
        // Set up tunable numbers for drive pids
        DrivePIDs.pidToPoseXkP.onChange(pidXController::setP);
        DrivePIDs.pidToPoseYkP.onChange(pidYController::setP);
        DrivePIDs.pidToPoseVelocity.onChange(
                value -> {
                    pidXController.setConstraints(
                            new TrapezoidProfile.Constraints(
                                    value, pidXController.getConstraints().maxAcceleration
                            )
                    );
                    pidYController.setConstraints(
                            new TrapezoidProfile.Constraints(
                                    value, pidYController.getConstraints().maxAcceleration
                            )
                    );
                }
        );
        DrivePIDs.pidToPoseAcceleration.onChange(
                value -> {
                    pidXController.setConstraints(
                            new TrapezoidProfile.Constraints(
                                    pidXController.getConstraints().maxVelocity, value
                            )
                    );
                    pidYController.setConstraints(
                            new TrapezoidProfile.Constraints(
                                    pidYController.getConstraints().maxVelocity, value
                            )
                    );
                }
        );
    }

    protected final Field2d field = new Field2d();

    private final SwerveSetpointGenerator swerveSetpointGenerator;
    private SwerveSetpoint prevSwerveSetpoint;
    private static final double MAX_STEER_VELOCITY_RADS_PER_SEC = 12.49;

    private boolean shouldUseMTSTDevs = false;

    private boolean isAligning = false, isAligned = false;

    private boolean shouldAutoSetHeading = false;

    public PhysicalSwerveDrivetrain() {
        SwerveDrivetrainConstants drivetrainConstants = TunerConstants.DrivetrainConstants;
        SwerveModuleConstants<?, ?, ?>[] modules = {
                TunerConstants.FrontLeft,
                TunerConstants.FrontRight,
                TunerConstants.BackLeft,
                TunerConstants.BackRight
        };

        this.drivetrain = new TunerSwerveDrivetrain(drivetrainConstants, modules);

        RobotConfig config = RobotConfigLoader.getOrLoadConfig();

        // Set up swerve setpoint generator
        swerveSetpointGenerator = new SwerveSetpointGenerator(config, MAX_STEER_VELOCITY_RADS_PER_SEC);
        prevSwerveSetpoint = new SwerveSetpoint(
                // When we start the robot, it should not be moving
                new ChassisSpeeds(),
                // Use the current state of the modules, which may not be zeroed
                drivetrain.getState().ModuleStates,
                // When we start the robot, it should not be moving
                DriveFeedforwards.zeros(config.numModules)
        );

        PathPlannerLogging.setLogActivePathCallback((poses) -> {
            field.getObject("Pathplanner Path").setPoses(poses);
        });

        headingController.enableContinuousInput(-Math.PI, Math.PI);
        drivetrain.setVisionMeasurementStdDevs(new Matrix<N3, N1>(Nat.N3(), Nat.N1(), new double[]{5, 5, 10}));

        RobotModeTriggers.autonomous().onTrue(Commands.runOnce(() -> this.setVisionStandardDeviations(0.5, 0.5, 10)));
        RobotModeTriggers.teleop().onTrue(Commands.runOnce(() -> this.setVisionStandardDeviations(10, 10, 1000)));
    }

    @Override
    public Command manualControl() {
        return applyRequest(
                () -> {
                    if (shouldAutoSetHeading
                //      && SubsystemManager.getInstance().getScoringSuperstructure().hasCoral()
                     )
                        return getFieldCentricFacingClosestReefRequest();
                    return getFieldCentricRequest();
                }
        );
    }

    private Rotation2d fieldCentricZeroRotation = Rotation2d.kZero;

    private SwerveRequest getFieldCentricRequest() {
        double rawForwards = Controls.Driver.SwerveForwardAxis.getAsDouble() * DriveConstants.CURRENT_MAX_ROBOT_MPS;
        double rawStrafe = -Controls.Driver.SwerveStrafeAxis.getAsDouble() * DriveConstants.CURRENT_MAX_ROBOT_MPS;
        double rawRotation = Controls.Driver.SwerveRotationAxis.getAsDouble() * DriveConstants.TELOP_ROTATION_SPEED;
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
                rawForwards,
                rawStrafe,
                rawRotation
        );

        if (rawForwards == 0 && rawStrafe == 0 && rawRotation == 0) return new SwerveRequest.SwerveDriveBrake();

        if (Controls.Driver.precisionTrigger.getAsBoolean()) {
            chassisSpeeds = chassisSpeeds.div(4.0);
        } else {
            chassisSpeeds = chassisSpeeds.times(getSwerveSpeedMultiplier());
        }

        Rotation2d currentRobotRotation = getPose().getRotation();

        SmartDashboard.putNumber("robot rotation", currentRobotRotation.getDegrees());
        SmartDashboard.putNumber("robot rotation offset", fieldCentricZeroRotation.getDegrees());

        SwerveSetpoint setpoint = swerveSetpointGenerator.generateSetpoint(
                prevSwerveSetpoint,
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        chassisSpeeds,
                        currentRobotRotation.minus(fieldCentricZeroRotation)
                ),
                0.02
        );
        prevSwerveSetpoint = setpoint;

        return new SwerveRequest.ApplyFieldSpeeds()
                .withDesaturateWheelSpeeds(true)
                .withSpeeds(
                        ChassisSpeeds.fromRobotRelativeSpeeds(
                                setpoint.robotRelativeSpeeds(),
                                currentRobotRotation
                        )
                )
                .withWheelForceFeedforwardsX(setpoint.feedforwards().robotRelativeForcesX())
                .withWheelForceFeedforwardsY(setpoint.feedforwards().robotRelativeForcesY());
    }

    private SwerveRequest getFieldCentricFacingClosestReefRequest() {
        double rawForwards = Controls.Driver.SwerveForwardAxis.getAsDouble() * DriveConstants.CURRENT_MAX_ROBOT_MPS;
        double rawStrafe = -Controls.Driver.SwerveStrafeAxis.getAsDouble() * DriveConstants.CURRENT_MAX_ROBOT_MPS;
        double rawRotation = Controls.Driver.SwerveRotationAxis.getAsDouble() * DriveConstants.TELOP_ROTATION_SPEED;
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
                rawForwards,
                rawStrafe,
                rawRotation
        );

        if (Controls.Driver.precisionTrigger.getAsBoolean()) {
            chassisSpeeds = chassisSpeeds.div(4.0);
        } else {
            chassisSpeeds = chassisSpeeds.times(getSwerveSpeedMultiplier());
        }

        if (DriverStationUtil.getAlliance() == Alliance.Blue) {
            chassisSpeeds.times(-1);
        }

        Pose2d nearestReefPose = DriveCommands.getClosestTarget(this::getPose);
        Rotation2d nearestReefPoseRotation = nearestReefPose.getRotation();

        var request = new SwerveRequest.FieldCentricFacingAngle()
                .withDesaturateWheelSpeeds(true)
                .withVelocityX(chassisSpeeds.vxMetersPerSecond)
                .withVelocityY(chassisSpeeds.vyMetersPerSecond)
                .withTargetDirection(nearestReefPoseRotation.plus(Rotation2d.k180deg));
        request.HeadingController = new PhoenixPIDController(headingController.getP(), 0, 0);
        request.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
        request.HeadingController.setTolerance(Radians.convertFrom(1, Degrees));
        if (rawForwards == 0 && rawStrafe == 0 && request.HeadingController.atSetpoint()) {
            return new SwerveRequest.SwerveDriveBrake();
        }
        return request;
    }

    @Override
    public Command resetHeadingToZero() {
        return runOnce(
                () -> {
                    fieldCentricZeroRotation = getPose().getRotation();
                }
        );
    }

    @Override
    public Command stop() {
        return applyRequest(SwerveRequest.SwerveDriveBrake::new);
    }

    @Override
    public Command _directlyMoveTo(Pose2d targetPose, Supplier<Pose2d> currentPose) {
        return Commands.runOnce(() -> {
                    Vector<N2> translationVector = getTranslationVector(super.currentAlignTarget);
                    pidXController.reset(translationVector.get(0));
                    pidYController.reset(translationVector.get(1));
                    pidXController.setGoal(0);
                    pidYController.setGoal(0);
                    SmartDashboard.putNumber("distanceThresholdMeters", 10);
                    field.getObject("Target Pose").setPose(super.currentAlignTarget);
                    isAligning = true;
                    isAligned = false;
                    shouldUseMTSTDevs = true;
                    Vision.addGlobalVisionMeasurements(this, true);
                }).andThen(applyRequest(
                        () -> {
                                Vision.addGlobalVisionMeasurements(this, shouldUseMTSTDevs);
                            Vector<N2> translationVector = getTranslationVector(targetPose);

                            var request = new SwerveRequest.RobotCentric();
                            double rotationalRate;
                            if (getCalculatedRotationFromAlign().isEmpty()) {
                                rotationalRate =
                                        headingController.calculate(
                                                getPose().getRotation().getRadians(),
                                                super.currentAlignTarget.getRotation().getRadians(),
                                                Timer.getFPGATimestamp()
                                        );
                            } else {
                                rotationalRate =
                                        headingController.calculate(
                                                getCalculatedRotationFromAlign().get().getRadians(),
                                                0,
                                                Timer.getFPGATimestamp()
                                        );
                            }

                            Vector<N2> setpointVector = invertTranslationVector(
                                    VecBuilder.fill(
                                            pidXController.getSetpoint().position,
                                            pidYController.getSetpoint().position
                                    ),
                                    super.currentAlignTarget.getRotation()
                            );
                            Translation2d toSetpointTranslation = new Translation2d(
                                    -setpointVector.get(0),
                                    -setpointVector.get(1)
                            );
                            Pose2d setpointPose = new Pose2d(
                                    targetPose.getTranslation().plus(toSetpointTranslation),
                                    targetPose.getRotation()
                            );
                            field.getObject("Setpoint Pose").setPose(setpointPose);

                            ChassisSpeeds reefRelativeSpeeds = new ChassisSpeeds(
                                -pidXController.calculate(translationVector.get(0)),
                                -pidYController.calculate(translationVector.get(1)),
                                rotationalRate
                            );

                            ChassisSpeeds robotRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                                reefRelativeSpeeds, 
                                getPose().getRotation().minus(targetPose.getRotation())
                            );

                            return request
                                    .withVelocityX(robotRelativeSpeeds.vxMetersPerSecond)
                                    .withVelocityY(robotRelativeSpeeds.vyMetersPerSecond)
                                    .withRotationalRate(rotationalRate);
                        }
                ).until(
                        getNearTargetPoseTrigger(targetPose)
                )).andThen(stop().withTimeout(0.1))
                .finallyDo(() -> {
                    setVisionStandardDeviations(5, 5, 10);
                    shouldUseMTSTDevs = false;
                    isAligning = false;
                    isAligned = false;
                });
    }

    private Vector<N2> getTranslationVector(Pose2d targetPose) {
        Vector<N2> targetTranslation = targetPose.getTranslation().toVector();
        Vector<N2> poseTranslation = getPose().getTranslation().toVector();
        Vector<N2> poseDiff = targetTranslation.minus(poseTranslation);
        Rotation2d rotation = targetPose.getRotation();
        Matrix<N2, N2> rotationMatrix = new Matrix<>(
                Nat.N2(),
                Nat.N2(),
                new double[]{
                        rotation.getCos(),
                        rotation.getSin(),
                        -rotation.getSin(),
                        rotation.getCos()
                }
        );
        Vector<N2> translationVector = new Vector<>(rotationMatrix.times(poseDiff));
        SmartDashboard.putNumber("Translation Forwards", translationVector.get(0));
        SmartDashboard.putNumber("Translation Side", translationVector.get(1));
        return translationVector;
    }

    private Vector<N2> invertTranslationVector(Vector<N2> vector, Rotation2d rotation) {
        Matrix<N2, N2> rotationMatrix = new Matrix<>(
                Nat.N2(),
                Nat.N2(),
                new double[]{
                        rotation.getCos(),
                        rotation.getSin(),
                        -rotation.getSin(),
                        rotation.getCos()
                }
        );
        Vector<N2> translationVector = new Vector<>(rotationMatrix.times(vector));
        SmartDashboard.putNumber("Translation Forwards", translationVector.get(0));
        SmartDashboard.putNumber("Translation Side", translationVector.get(1));
        return translationVector;
    }

    public Trigger getNearTargetPoseTrigger(Pose2d targetPose) {
        return new Trigger(() -> nearTargetPose(targetPose));
    }

    public boolean nearTargetPose(Pose2d targetPose) {
        return MathUtil.isNear(targetPose.getX(), getPose().getX(), 0.5)//0.01
                && MathUtil.isNear(targetPose.getY(), getPose().getY(), 0.5)//0.01
                && MathUtil.isNear(targetPose.getRotation().getDegrees(), getPose().getRotation().getDegrees(), 20);
    }

    @Override
    public Command fineTuneUsingLaserCANCommand(Pose2d targetPose) {
        return Commands.runOnce(() -> {
            Vector<N2> translationVector = getTranslationVector(targetPose);
            pidYController.reset(translationVector.get(1));
            pidYController.setTolerance(0.05);
            shouldUseMTSTDevs = true;
        }).andThen(
                applyRequest(
                        () -> {
                            Vector<N2> translationVector = getTranslationVector(targetPose);
                            double laserCANAlignOutput = SubsystemManager.getInstance().getLasercanAlign().getOutput();
                            double rotationRadians = getCalculatedRotationFromAlign().orElseGet(
                                    Rotation2d::new
                            ).getRadians();
                            double rotationOutput =
                                    headingController.calculate(rotationRadians, 0, Timer.getFPGATimestamp());
                            ChassisSpeeds robotCentricSpeeds = new ChassisSpeeds(
                                    -laserCANAlignOutput,
                                    -pidYController.calculate(translationVector.get(1)),
                                    rotationOutput
                            );
                            SmartDashboard.putNumber("Rotation Output", rotationOutput);
                            return new SwerveRequest.RobotCentric()
                                    .withVelocityX(robotCentricSpeeds.vxMetersPerSecond)
                                    .withVelocityY(robotCentricSpeeds.vyMetersPerSecond)
                                    .withRotationalRate(rotationOutput);
                        }
                )
        ).until(
                new Trigger(() -> {
                        boolean res = MathUtil.isNear(
                                getDistanceFromReefFace(), 
                                LasercanAlign.alignDistance_mm, 
                                10      
                        ) && MathUtil.isNear(
                                getCalculatedRotationFromAlign().orElseGet(() -> new Rotation2d()).getDegrees(), 
                                0, 
                                1
                        ) && pidYController.atGoal();
                        SmartDashboard.putBoolean("PIDY at setpoint", pidYController.atGoal());
                        return res;
                }).debounce(0.375)
        ).finallyDo(
                () -> shouldUseMTSTDevs = false
        );
    }

    @Override
    public Command pathToPoseCommand(Pose2d targetPose) {
        return Commands.runOnce(
                () -> shouldUseMTSTDevs = true
        ).andThen(
                getPathFromWaypoint(targetPose)
        ).finallyDo(
                () -> shouldUseMTSTDevs = false
        );
    }

    private Command getPathFromWaypoint(Pose2d waypoint) {
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
                new Pose2d(getPose().getTranslation(), getPathVelocityHeading(getChassisSpeeds(), waypoint)),
                waypoint
        );
        PathConstraints constraints = new PathConstraints(
                1, 2,
                2 * Math.PI, 2 * Math.PI
        );
        PathPlannerPath path = new PathPlannerPath(
                waypoints,
                constraints,
                new IdealStartingState(getVelocityMagnitude(getChassisSpeeds()), getPose().getRotation()),
                new GoalEndState(0.0, waypoint.getRotation())
        );

        path.preventFlipping = true;

        return AutoBuilder.followPath(path);
    }

    private Rotation2d getPathVelocityHeading(ChassisSpeeds chassisSpeeds, Pose2d targetPose) {
        if (getVelocityMagnitude(chassisSpeeds).in(MetersPerSecond) < 0.25) {
            var diff = targetPose.minus(getPose()).getTranslation();
            return (diff.getNorm() < 0.01) ? targetPose.getRotation() : diff.getAngle();
        }
        return new Rotation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
    }

    private LinearVelocity getVelocityMagnitude(ChassisSpeeds chassisSpeeds) {
        return MetersPerSecond.of(new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond).getNorm());
    }

    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param requestSupplier Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> setControl(requestSupplier.get()));
    }

    @Override
    public void followPath(SwerveSample sample) {

        var pose = getPose();

        var targetSpeeds = sample.getChassisSpeeds();
        targetSpeeds.vxMetersPerSecond += pathXController.calculate(pose.getX(), sample.x);
        targetSpeeds.vyMetersPerSecond += pathYController.calculate(pose.getY(), sample.y);
        targetSpeeds.omegaRadiansPerSecond += pathHeadingController.calculate(
                pose.getRotation().getRadians(), sample.heading
        );

        drivetrain.setControl(new SwerveRequest.ApplyFieldSpeeds()
                .withSpeeds(targetSpeeds)
                .withWheelForceFeedforwardsX(sample.moduleForcesX())
                .withWheelForceFeedforwardsY(sample.moduleForcesY())
        );
    }

    @Override
    public void setControl(SwerveRequest control) {
        drivetrain.setControl(control);
    }

    @Override
    public void periodic() {
        //update profiled pid controllers
        pidXController.calculate(getPose().getX());
        pidYController.calculate(getPose().getY());

        // Update the operator perspective if needed
        if (!didApplyOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                drivetrain.setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red
                                ? DriveConstants.RedAlliancePerspectiveRotation
                                : DriveConstants.BlueAlliancePerspectiveRotation
                );
                didApplyOperatorPerspective = true;
            });
        }

        Vision.addGlobalVisionMeasurements(this, shouldUseMTSTDevs);

        // Update robot pose
        field.setRobotPose(getPose());

        // Update field on dashboard
        SmartDashboard.putData("Field2D", field);
        SignalLogger.writeDouble("Rotate Velocity", drivetrain.getPigeon2().getAngularVelocityZDevice().getValueAsDouble());
        SignalLogger.writeDouble("Rotate Position", drivetrain.getPigeon2().getYaw().getValueAsDouble());

        SmartDashboard.putNumber("Current heading", getPose().getRotation().getDegrees());
        SmartDashboard.putBoolean("use mt1 stdevs", shouldUseMTSTDevs);

        SmartDashboard.putBoolean("Aligned", isAligned());

        getTranslationVector(DriveCommands.getClosestTarget(this::getPose));

        getCalculatedRotationFromAlign().ifPresent(
                rotation -> SmartDashboard.putNumber("Reef Rotation Degrees", rotation.getDegrees())
        );
        SmartDashboard.putNumber("Left LC", SubsystemManager.getInstance().getLasercanAlign().getLeftMeasurement());
        SmartDashboard.putNumber("Right LC", SubsystemManager.getInstance().getLasercanAlign().getRightMeasurement());

        double rotationRadians = getCalculatedRotationFromAlign().orElseGet(
                Rotation2d::new
        ).getRadians();
        double rotationOutput =
                headingController.calculate(rotationRadians, 0, Timer.getFPGATimestamp());
        SmartDashboard.putNumber("Rotation Output", rotationOutput);
        SmartDashboard.putNumber("Distance from Reef Face", getDistanceFromReefFace());
        SmartDashboard.putBoolean("Snap to Reef", shouldAutoSetHeading);

        Arrays.stream(Limelights.values()).parallel().forEach(
                limelight -> {
                    LimelightHelpers.setRobotOrientation(
                            limelight.getName(),
                            getPose().getRotation().minus(fieldCentricZeroRotation).plus(
                                    DriverStationUtil.getAlliance() == Alliance.Red
                                            ? Rotation2d.k180deg
                                            : Rotation2d.kZero
                            ).getDegrees()
                    );
                }
        );
    }

    @Override
    public Pose2d getPose() {
        return drivetrain.getState().Pose;
    }

    @Override
    public ChassisSpeeds getChassisSpeeds() {
        return drivetrain.getState().Speeds;
    }

    @Override
    public double getAccelerationInGs() {
        return Math.hypot(
                drivetrain.getPigeon2().getAccelerationX().getValueAsDouble(),
                drivetrain.getPigeon2().getAccelerationY().getValueAsDouble()
        );
    }

    @Override
    public double getDistanceFromReefFace() {
        return SubsystemManager.getInstance().getLasercanAlign().getDistance_mm();
    }

    @Override
    public Optional<Rotation2d> getCalculatedRotationFromAlign() {
        double leftMeasurement, rightMeasurement;
        leftMeasurement = SubsystemManager.getInstance().getLasercanAlign().getLeftMeasurement();
        rightMeasurement = SubsystemManager.getInstance().getLasercanAlign().getRightMeasurement();
        if (leftMeasurement == -1 || rightMeasurement == -1) {
            return Optional.empty();
        } else {
            return Optional.of(
                    Rotation2d.fromRadians(
                            Math.tan((leftMeasurement - rightMeasurement) / DriveConstants.laserCanDistanceMM.in(Millimeters))
                    )
            );
        }
    }

    @Override
    public void resetPose(Pose2d pose) {
        drivetrain.resetPose(pose);
    }

    @Override
    public void addVisionMeasurement(Pose2d pose, double timestamp) {
        drivetrain.addVisionMeasurement(pose, timestamp);
    }

    @Override
    public void setVisionStandardDeviations(double xStdDev, double yStdDev, double rotStdDev) {
        drivetrain.setVisionMeasurementStdDevs(new Matrix<N3, N1>(Nat.N3(), Nat.N1(), new double[]{xStdDev, yStdDev, rotStdDev}));
    }

    protected Field2d getField() {
        return field;
    }

    @Override
    public boolean isAligned() {
        return isAligning && isAligned;
    }

    @Override
    public Command toggleAutoHeading() {
        return Commands.runOnce(
                () -> shouldAutoSetHeading = !shouldAutoSetHeading
        );
    }
}