package frc.robot.subsystems.drive;

import choreo.Choreo.TrajectoryLogger;
import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.network.LimelightHelpers;
import frc.robot.commands.AutoRoutines;
import frc.robot.constants.Controls;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.IDs;
import frc.robot.subsystems.drive.constants.DriveConstants;
import frc.robot.subsystems.drive.constants.TunerConstants;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Objects;
import java.util.Optional;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;

public class CommandSwerveDrivetrain extends TunerConstants.TunerSwerveDrivetrain implements Subsystem {
    private static CommandSwerveDrivetrain instance;

    private static final double SIM_LOOP_PERIOD = 0.005; // 5 ms
    private Notifier simNotifier = null;
    private double lastSimTime;

    // TODO: Shouldn't these Rotation2ds go in some sort of constants?
    //  -- Jonathan
    private static final Rotation2d BlueAlliancePerspectiveRotation = Rotation2d.kZero;
    private static final Rotation2d RedAlliancePerspectiveRotation = Rotation2d.k180deg;

    private boolean hasAppliedOperatorPerspective = false;

    // TODO: Can the SendableChooser be elsewhere? Extract this to a class probably
    //  -- Jonathan
    private SendableChooser<Pose2d> m_startPositionChooser = new SendableChooser<>();

    // TODO: Can the construction of these (the SwerveRequests) be inlined? Does the identity of the request object
    //  matter or not? As far as I can tell, the request object very cheap to construct and performance is not an issue.
    //  -- Jonathan
    private final SwerveRequest.ApplyFieldSpeeds fieldSpeedsRequest = new SwerveRequest.ApplyFieldSpeeds();
    private final SwerveRequest.ApplyRobotSpeeds robotSpeedsRequest = new SwerveRequest.ApplyRobotSpeeds();
    private final SwerveRequest.FieldCentricFacingAngle fieldCentricFacingAngleRequest = new SwerveRequest.FieldCentricFacingAngle();

    private final PIDController pathXController = new PIDController(10, 0, 0);
    private final PIDController pathYController = new PIDController(10, 0, 0);
    private final PIDController pathThetaController = new PIDController(7, 0, 0);

    private final Field2d field = new Field2d();

    // TODO: Can we move sysid stuff to its own class? This class is already very cluttered.
    //  -- Jonathan
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();


    private final SysIdRoutine sysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,
            Volts.of(4),
            null,
            state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> setControl(m_translationCharacterization.withVolts(output)),
            null,
            this
        )
    );


    private final SysIdRoutine sysIdRoutineSteer = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(7), // Use dynamic voltage of 7 V
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> setControl(m_steerCharacterization.withVolts(volts)),
            null,
            this
        )
    );


    private final SysIdRoutine sysIdRoutineRotation = new SysIdRoutine(
        new SysIdRoutine.Config(
            Volts.of(Math.PI / 6).per(Second),
            Volts.of(Math.PI),
            null,
            state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> {
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
            },
            null,
            this
        )
    );

    private final SysIdRoutine sysIdRoutineToApply = sysIdRoutineRotation;

    private final AutoFactory autoFactory;
    private final AutoRoutines autoRoutines;

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
        autoFactory = createAutoFactory();
        autoRoutines = new AutoRoutines(autoFactory);
        configureAutoBuilder();
        fieldCentricFacingAngleRequest.HeadingController.setPID(
            28.48,
            0,
            1.1466
        );
        m_startPositionChooser.setDefaultOption("DEFAULT", new Pose2d());
        Arrays.stream(FieldConstants.AutonStartingPositions.values()).forEach(
            position -> m_startPositionChooser.addOption(position.name(), position.Pose)
        );
        SmartDashboard.putData("Selected Reset Position", m_startPositionChooser);
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
            false,
            this,
            trajLogger
        );
    }

    public SwerveRequest fieldCentricRequestSupplier() {
        double forwards = Controls.Driver.SwerveForwardAxis.getAsDouble() * DriveConstants.CURRENT_MAX_ROBOT_MPS;
        double strafe = -Controls.Driver.SwerveStrafeAxis.getAsDouble() * DriveConstants.CURRENT_MAX_ROBOT_MPS;
        double rotation = Controls.Driver.SwerveRotationAxis.getAsDouble() * DriveConstants.CURRENT_MAX_ROBOT_MPS;
        if (Controls.Driver.precisionTrigger.getAsBoolean()) {
            forwards /= 4;
            strafe /= 4;
            rotation /= 4;
        }
        return fieldSpeedsRequest.withDesaturateWheelSpeeds(true)
            .withSpeeds(
                new ChassisSpeeds(
                    forwards,
                    strafe,
                    rotation
                )
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

        setControl(
            fieldSpeedsRequest.withSpeeds(targetSpeeds)
                .withWheelForceFeedforwardsX(sample.moduleForcesX())
                .withWheelForceFeedforwardsY(sample.moduleForcesY())
        );
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutineToApply.dynamic(direction);
    }

    @Override
    public void periodic() {
        if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? RedAlliancePerspectiveRotation
                        : BlueAlliancePerspectiveRotation
                );
                hasAppliedOperatorPerspective = true;
            });
        }
        field.setRobotPose(getState().Pose);
        SmartDashboard.putData("Field2D", field);

        if (RobotBase.isReal()) Arrays.stream(IDs.Limelights.values()).forEach(
            limelight -> {
                Optional<Pose2d> measurement = Optional.of(
                    DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                        ? LimelightHelpers.getBotPose2d_wpiBlue(limelight.getName())
                        : LimelightHelpers.getBotPose2d_wpiRed(limelight.getName())
                );
                measurement = measurement.isPresent()
                                ? (measurement.get().getX() == 0 || measurement.get().getY() == 0
                                    ? Optional.empty()
                                    : measurement)
                                : Optional.empty();
                measurement.ifPresent(pose -> addVisionMeasurement(pose, Utils.getCurrentTimeSeconds()));
            }
        );
    }

    private void startSimThread() {
        lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        simNotifier = new Notifier(() -> {
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
                (speeds, feedforwards) -> setControl(
                    robotSpeedsRequest.withSpeeds(speeds)
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
                ArrayList<Trajectory.State> states = new ArrayList<>();
                if (poses.size() > 1) {
                    Pose2d lastPose = poses.get(0);
                    double t = 0;
                    for (var pose : poses.subList(1, poses.size())) {
                        Pose2d delta = new Pose2d(pose.getTranslation().minus(lastPose.getTranslation()), pose.getRotation().minus(lastPose.getRotation()));
                        double curvature = delta.getRotation().getRadians() / delta.getTranslation().getNorm();
                        states.add(new Trajectory.State(t, delta.getX(), delta.getY(), pose, curvature));
                        t += 0.02;
                    }
                } else {
                    states.add(new Trajectory.State(
                        0,
                        0,
                        0,
                        new Pose2d(-100, -100, new Rotation2d()),
                        0));
                }
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

    public Pose2d getDashboardSelectedResetPose() {
        return m_startPositionChooser.getSelected();
    }
}