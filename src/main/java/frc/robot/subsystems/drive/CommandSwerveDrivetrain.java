package frc.robot.subsystems.drive;

import choreo.trajectory.SwerveSample;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.DriverStationHelpers;
import frc.robot.constants.Controls;
import frc.robot.subsystems.drive.constants.DriveConstants;
import frc.robot.subsystems.drive.constants.DrivePIDs;
import frc.robot.subsystems.drive.constants.TunerConstants;
import frc.robot.subsystems.drive.constants.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.subsystems.scoring.elevator.ElevatorSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

import java.util.Objects;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

public class CommandSwerveDrivetrain extends Drivetrain {
    private final TunerSwerveDrivetrain drivetrain;

    private static final double SIM_LOOP_PERIOD = 0.005; // 5 ms

    private boolean didApplyOperatorPerspective = false;

    private final PhoenixPIDController headingController = new PhoenixPIDController(28.48, 0, 1.1466);
    private final PIDController
        pathXController = new PIDController(12, 0, 0),
        pathYController = new PIDController(12, 0, 0),
        pathHeadingController = new PIDController(7, 0, 0),
        pidXController = new PIDController(1, 0, 0),
        pidYController = new PIDController(1, 0, 0);

    {
        pathHeadingController.enableContinuousInput(-Math.PI, Math.PI);
        // Set up tunable numbers for drive pids
        DrivePIDs.pidToPoseXkP.onChange(pidXController::setP);
        DrivePIDs.pidToPoseYkP.onChange(pidYController::setP);
    }

    private final Field2d field = new Field2d();

    private final SwerveSetpointGenerator swerveSetpointGenerator;
    private SwerveSetpoint prevSwerveSetpoint;
    private static final double MAX_STEER_VELOCITY_RADS_PER_SEC = 12.49;

    private static CommandSwerveDrivetrain instance;

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

        // Start the simulation thread if needed
        if (Utils.isSimulation()) startSimThread();
    }

    /**
     * Get a SwerveRequest for field centric movement controlled by the human drivers. This should be called
     * periodically in a command to always get the latest values from the controllers.
     *
     * @return The swerve request.
     */
    public SwerveRequest getFieldCentricRequest() {
        double rawForwards = Controls.Driver.SwerveForwardAxis.getAsDouble() * DriveConstants.CURRENT_MAX_ROBOT_MPS;
        double rawStrafe = -Controls.Driver.SwerveStrafeAxis.getAsDouble() * DriveConstants.CURRENT_MAX_ROBOT_MPS;
        double rawRotation = Controls.Driver.SwerveRotationAxis.getAsDouble() * DriveConstants.TELOP_ROTATION_SPEED;
        double allianceBasedDirection = DriverStationHelpers.getAlliance() == Alliance.Blue ? 1 : -1;
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
            allianceBasedDirection * rawForwards,
            allianceBasedDirection * rawStrafe,
            rawRotation
        );

        if (Controls.Driver.precisionTrigger.getAsBoolean()) {
            chassisSpeeds = chassisSpeeds.div(4.0);
        } else {
            chassisSpeeds = chassisSpeeds.times(getSwerveSpeedMultiplier());
        }

        SwerveSetpoint setpoint = swerveSetpointGenerator.generateSetpoint(
            prevSwerveSetpoint,
            ChassisSpeeds.fromFieldRelativeSpeeds(
                chassisSpeeds,
                Rotation2d.fromRadians(drivetrain.getPigeon2().getYaw().getValue().in(Radians))
            ),
            0.02
        );
        prevSwerveSetpoint = setpoint;

        return new SwerveRequest.ApplyFieldSpeeds()
            .withDesaturateWheelSpeeds(true)
            .withSpeeds(
                ChassisSpeeds.fromRobotRelativeSpeeds(
                    setpoint.robotRelativeSpeeds(),
                    Rotation2d.fromRadians(drivetrain.getPigeon2().getYaw().getValue().in(Radians))
                )
            )
            .withWheelForceFeedforwardsX(setpoint.feedforwards().robotRelativeForcesX())
            .withWheelForceFeedforwardsY(setpoint.feedforwards().robotRelativeForcesY());
    }

    /**
     * Returns a command that resets the heading of the robot such that the current heading is zero
     *
     * @return Command to run
     */
    public Command resetPigeon() {
        return Commands.runOnce(
            () -> drivetrain.getPigeon2().setYaw(
                Angle.ofBaseUnits(
                    0,
                    Degrees
                )
            )
        );
    }

    /**
     * Returns a command that stops the swerve drive
     *
     * @return Command to run
     */
    public Command stop() {
        return applyRequest(SwerveRequest.SwerveDriveBrake::new);
    }

    /**
     * Returns a command that makes the robot pathfind to the specified pose
     *
     * @param targetPose The pose to move to
     * @return Command to run
     */
    public Command pathfindTo(Pose2d targetPose) {
        double driveBaseRadius = 0;
        for (var moduleLocation : drivetrain.getModuleLocations()) {
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

    /**
     * Returns a command that moves the robot to the specified pose under PID control, without pathfinding
     *
     * @param targetPose The pose to move to
     * @return Command to run
     */
    public Command directlyMoveTo(Pose2d targetPose) {
        return new InstantCommand(() -> {
            pidXController.setSetpoint(targetPose.getX());
            pidYController.setSetpoint(targetPose.getY());
        }).andThen(applyRequest(
            () -> {
                double pidXOutput = -pidXController.calculate(getPose().getX());
                double pidYOutput = -pidYController.calculate(getPose().getY());

                var request = new SwerveRequest.FieldCentricFacingAngle();
                request.HeadingController = headingController;

                return request
                    .withVelocityX(pidXOutput)
                    .withVelocityY(pidYOutput)
                    // Michael says not sure why the 180-degree rotation is needed, but it just works
                    .withTargetDirection(targetPose.getRotation().plus(Rotation2d.k180deg));
            }
        ).until(
            () -> MathUtil.isNear(targetPose.getX(), getPose().getX(), 0.025)
                && MathUtil.isNear(targetPose.getY(), getPose().getY(), 0.025)
        ));
    }

    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param requestSupplier Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> drivetrain.setControl(requestSupplier.get()));
    }

    /**
     * Follows the given field-centric path sample with PID.
     *
     * @param sample Sample along the path to follow
     */
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
    public void periodic() {
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
        // Update vision
        VisionSubsystem.getInstance().getVisionResults().forEach(
            visionResult -> drivetrain.addVisionMeasurement(
                visionResult.getVisionPose(),
                visionResult.getTimestamp()
            )
        );
        // Update robot pose
        field.setRobotPose(getPose());

        // Update field on dashboard
        SmartDashboard.putData("Field2D", field);
    }

    /**
     * Runs the thread that sets the simulation state
     */
    @SuppressWarnings("resource")
    private void startSimThread() {
        new Notifier(new Runnable() {
            double lastUpdateTimeSeconds = Utils.getCurrentTimeSeconds();

            @Override
            public void run() {
                // Calculate the actual time elapsed since the last invocation
                double deltaTimeSeconds = Utils.getCurrentTimeSeconds() - lastUpdateTimeSeconds;
                lastUpdateTimeSeconds += deltaTimeSeconds;

                drivetrain.updateSimState(
                    deltaTimeSeconds,
                    RobotController.getBatteryVoltage()
                );
            }
        }).startPeriodic(SIM_LOOP_PERIOD); // Run the simulation at a faster rate so PID behaves more reasonably
    }

    /**
     * Gets the estimated position of the robot as coordinates on the field
     * 
     * @return position as Pose2d
     */
    public Pose2d getPose() {
        return drivetrain.getState().Pose;
    }

    /**
     * Gets the chassis speeds of the robot
     * 
     * @return chassis speeds as ChassisSpeeds
     */
    public ChassisSpeeds getChassisSpeeds() {
        return drivetrain.getState().Speeds;
    }

    /**
     * Estimates the acceleration of the robot in gs
     * 
     * @return acceleration in gs as double
    */
    public double getAccelerationInGs() {
        return Math.hypot(
            drivetrain.getPigeon2().getAccelerationX().getValueAsDouble(),
            drivetrain.getPigeon2().getAccelerationY().getValueAsDouble()
        );
    }

    /**
     * Slows the robot swerve when the elevator is raised. Reduction is proportional to the proportional height of the elevator.
     * 
     * @return multiplier as double
     */
    public double getSwerveSpeedMultiplier() {
        return 1 - Math.pow(ElevatorSubsystem.getInstance().getCurrentProportion(), 3) / 2;
    }
}