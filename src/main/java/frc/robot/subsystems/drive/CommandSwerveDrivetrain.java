package frc.robot.subsystems.drive;

import choreo.Choreo.TrajectoryLogger;
import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.lib.DriverStationHelpers;
import frc.robot.commands.AutoRoutines;
import frc.robot.constants.Controls;
import frc.robot.subsystems.drive.constants.DriveConstants;
import frc.robot.subsystems.drive.constants.DrivePIDs;
import frc.robot.subsystems.drive.constants.TunerConstants;
import frc.robot.subsystems.vision.VisionSubsystem;

import java.util.Objects;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

public class CommandSwerveDrivetrain extends TunerConstants.TunerSwerveDrivetrain implements Subsystem {
    private static CommandSwerveDrivetrain instance;

    private static final double SIM_LOOP_PERIOD = 0.005; // 5 ms
    private double lastSimTime;

    private boolean hasAppliedOperatorPerspective = false;

    private final PIDController pathXController = new PIDController(12, 0, 0);
    private final PIDController pathYController = new PIDController(12, 0, 0);
    private final PIDController pathThetaController = new PIDController(7, 0, 0);

    private final PIDController pidXController = new PIDController(1, 0, 0);
    private final PIDController pidYController = new PIDController(1, 0, 0);

    private final PhoenixPIDController headingController = new PhoenixPIDController(0, 0, 0);

    private final Field2d field = new Field2d();

    private final AutoFactory autoFactory;
    private final AutoRoutines autoRoutines;

    private final SwerveSetpointGenerator swerveSetpointGenerator;
    private final RobotConfig config;

    private SwerveSetpoint prevSetpoint;

    public static CommandSwerveDrivetrain getInstance() {
        return instance = Objects.requireNonNullElseGet(instance, TunerConstants::createDrivetrain);
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
     * @param modules             Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            throw new RuntimeException("Swerve Config Failed");
        }
        ChassisSpeeds currentSpeeds = getState().Speeds; // Method to get current robot-relative chassis speeds
        SwerveModuleState[] currentStates = getState().ModuleStates; // Method to get the current swerve module states
        swerveSetpointGenerator = new SwerveSetpointGenerator(config, 12.49);
        prevSetpoint = new SwerveSetpoint(currentSpeeds, currentStates, DriveFeedforwards.zeros(config.numModules));
        prevSetpoint = new SwerveSetpoint(new ChassisSpeeds(), getState().ModuleStates, DriveFeedforwards.zeros(config.numModules));
        autoFactory = createAutoFactory();
        autoRoutines = new AutoRoutines(autoFactory);
        configureAutoBuilder();

        headingController.setPID(
            28.48,
            0,
            1.1466
        );
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If
     *                                unspecified or set to 0 Hz, this is 250 Hz on
     *                                CAN FD, and 100 Hz on CAN 2.0.
     * @param modules                 Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            throw new RuntimeException("Swerve Config Failed");
        }
        ChassisSpeeds currentSpeeds = getState().Speeds; // Method to get current robot-relative chassis speeds
        SwerveModuleState[] currentStates = getState().ModuleStates; // Method to get the current swerve module states
        swerveSetpointGenerator = new SwerveSetpointGenerator(config, 12.49);
        prevSetpoint = new SwerveSetpoint(currentSpeeds, currentStates, DriveFeedforwards.zeros(config.numModules));
        autoFactory = createAutoFactory();
        autoRoutines = new AutoRoutines(autoFactory);
        configureAutoBuilder();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants       Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
     *                                  unspecified or set to 0 Hz, this is 250 Hz on
     *                                  CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation The standard deviation for odometry calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param visionStandardDeviation   The standard deviation for vision calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param modules                   Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        Matrix<N3, N1> odometryStandardDeviation,
        Matrix<N3, N1> visionStandardDeviation,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            throw new RuntimeException("Swerve Config Failed");
        }
        ChassisSpeeds currentSpeeds = getState().Speeds; // Method to get current robot-relative chassis speeds
        SwerveModuleState[] currentStates = getState().ModuleStates; // Method to get the current swerve module states
        swerveSetpointGenerator = new SwerveSetpointGenerator(config, 12.49);
        prevSetpoint = new SwerveSetpoint(currentSpeeds, currentStates, DriveFeedforwards.zeros(config.numModules));
        autoFactory = createAutoFactory();
        autoRoutines = new AutoRoutines(autoFactory);
        configureAutoBuilder();
    }

    /**
     * Creates a new auto factory for this drivetrain.
     *
     * @return AutoFactory for this drivetrain
     */
    public AutoFactory createAutoFactory() {
        return createAutoFactory((sample, isStart) -> {});
    }

    /**
     * Creates a new auto factory for this drivetrain with the given
     * trajectory logger.
     *
     * @param trajLogger Logger for the trajectory
     * @return AutoFactory for this drivetrain
     */
    public AutoFactory createAutoFactory(TrajectoryLogger<SwerveSample> trajLogger) {
        return new AutoFactory(
            () -> getState().Pose,
            this::resetPose,
            this::followPath,
            true,
            this,
            trajLogger
        );
    }


    public Command stopCommand() {
        return applyRequest(SwerveRequest.SwerveDriveBrake::new);
    }

    public SwerveRequest fieldCentricRequestSupplier() {
        double forwards = (DriverStationHelpers.getAlliance() == Alliance.Blue ? 1 : -1) * Controls.Driver.SwerveForwardAxis.getAsDouble() * DriveConstants.CURRENT_MAX_ROBOT_MPS;
        double strafe = (DriverStationHelpers.getAlliance() == Alliance.Blue ? 1 : -1) * -Controls.Driver.SwerveStrafeAxis.getAsDouble() * DriveConstants.CURRENT_MAX_ROBOT_MPS;
        double rotation = Controls.Driver.SwerveRotationAxis.getAsDouble() * DriveConstants.TELOP_ROTATION_SPEED;
        if (Controls.Driver.precisionTrigger.getAsBoolean()) {
            forwards /= 4;
            strafe /= 4;
            rotation /= 4;
        }
        SwerveSetpoint newSetpoint = swerveSetpointGenerator.generateSetpoint(
            prevSetpoint,
            ChassisSpeeds.fromFieldRelativeSpeeds(
                new ChassisSpeeds(
                    forwards,
                    strafe,
                    rotation
                ),
                Rotation2d.fromRadians(getPigeon2().getYaw().getValue().in(Radians))
            ),
            0.02
        );
        prevSetpoint = newSetpoint;
        return new SwerveRequest.ApplyFieldSpeeds()
            .withDesaturateWheelSpeeds(true)
            .withSpeeds(
                ChassisSpeeds.fromRobotRelativeSpeeds(
                    newSetpoint.robotRelativeSpeeds(),
                    Rotation2d.fromRadians(getPigeon2().getYaw().getValue().in(Radians))
                )
            )
            .withWheelForceFeedforwardsX(
                newSetpoint.feedforwards().robotRelativeForcesX()
            )
            .withWheelForceFeedforwardsY(
                newSetpoint.feedforwards().robotRelativeForcesY()
            );
    }

    public Command pathfindCommand(Pose2d targetPose) {
        double driveBaseRadius = 0;
        for (var moduleLocation : getModuleLocations()) {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        }
        PathConstraints constraints = new PathConstraints(
            10, 5,
            2 * Math.PI, 2 * Math.PI
        );
        return AutoBuilder.pathfindToPoseFlipped(
            targetPose,
            constraints
        );
    }

    public Command pidToPoseCommand(Pose2d targetPose) {
        return applyRequest(
            () -> {
                pidXController.setSetpoint(targetPose.getX());
                pidYController.setSetpoint(targetPose.getY());
                double pidXOutput = -pidXController.calculate(getState().Pose.getX());
                double pidYOutput = -pidYController.calculate(getState().Pose.getY());

                var request = new SwerveRequest.FieldCentricFacingAngle();
                request.HeadingController = headingController;

                return request
                    .withTargetDirection(
                        targetPose.getRotation().plus(Rotation2d.fromDegrees(180))
                    )
                    .withVelocityX(
                        pidXOutput
                    )
                    .withVelocityY(
                        pidYOutput
                    );
            }
        ).until(
            () -> MathUtil.isNear(targetPose.getX(), getState().Pose.getX(), 0.025)
                && MathUtil.isNear(targetPose.getY(), getState().Pose.getY(), 0.025)
        );
    }

    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param requestSupplier Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Follows the given field-centric path sample with PID.
     *
     * @param sample Sample along the path to follow
     */
    public void followPath(SwerveSample sample) {
        pathThetaController.enableContinuousInput(-Math.PI, Math.PI);

        var pose = getState().Pose;

        var targetSpeeds = sample.getChassisSpeeds();
        targetSpeeds.vxMetersPerSecond += pathXController.calculate(
            pose.getX(), sample.x
        );
        targetSpeeds.vyMetersPerSecond += pathYController.calculate(
            pose.getY(), sample.y
        );
        targetSpeeds.omegaRadiansPerSecond += pathThetaController.calculate(
            pose.getRotation().getRadians(), sample.heading
        );

        setControl(new SwerveRequest.ApplyFieldSpeeds()
            .withSpeeds(targetSpeeds)
            .withWheelForceFeedforwardsX(sample.moduleForcesX())
            .withWheelForceFeedforwardsY(sample.moduleForcesY())
        );
    }

    @Override
    public void periodic() {
        updatePIDs();
        if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? DriveConstants.RedAlliancePerspectiveRotation
                        : DriveConstants.BlueAlliancePerspectiveRotation
                );
                hasAppliedOperatorPerspective = true;
            });
        }
        VisionSubsystem
            .getInstance()
            .getVisionResults()
            .forEach(
                visionResult -> addVisionMeasurement(
                    visionResult.getVisionPose(),
                    visionResult.getTimestamp())
            );
        field.setRobotPose(getState().Pose);
        SmartDashboard.putData("Field2D", field);
    }

    private void updatePIDs() {
        pidXController.setP(DrivePIDs.pidToPoseXkP.get());
        pidYController.setP(DrivePIDs.pidToPoseYkP.get());
        pidXController.calculate(getState().Pose.getX());
        pidYController.calculate(getState().Pose.getY());
    }

    private void startSimThread() {
        lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        /* use the measured time delta, get battery voltage from WPILib */
        @SuppressWarnings("resource")
        Notifier simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - lastSimTime;
            lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        simNotifier.startPeriodic(SIM_LOOP_PERIOD);
    }

    private void configureAutoBuilder() {
        try {
            var config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                () -> getState().Pose,   // Supplier of current robot pose
                this::resetPose,         // Consumer for seeding pose against auto
                () -> getState().Speeds, // Supplier of current robot speeds
                // Consumer of ChassisSpeeds and feedforwards to drive the robot
                (speeds, feedforwards) -> setControl(new SwerveRequest.ApplyRobotSpeeds()
                    .withSpeeds(speeds)
                    .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                    .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
                ),
                new PPHolonomicDriveController(
                    // PID constants for translation
                    new PIDConstants(10, 0, 0),
                    // PID constants for rotation
                    new PIDConstants(7, 0, 0)
                ),
                config,
                // Assume the path needs to be flipped for Red vs Blue, this is normally the case
                () -> false,
                this // Subsystem for requirements
            );
            PathPlannerLogging.setLogActivePathCallback((poses) -> {
                field.getObject("Pathplanner Path").setPoses(poses);
            });
        } catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
        }
    }

    public AutoFactory getAutoFactory() {
        return autoFactory;
    }

    public AutoRoutines getAutoRoutines() {
        return autoRoutines;
    }

    public Command resetPigeonCommand() {
        return Commands.runOnce(
            () -> {
                getPigeon2().setYaw(
                    Angle.ofBaseUnits(
                        0,
                        Degrees
                    )
                );
            }
        );
    }

    public double getAccelerationInGs() {
        return Math.hypot(
            getPigeon2().getAccelerationX().getValueAsDouble(),
            getPigeon2().getAccelerationY().getValueAsDouble()
        );
    }
}