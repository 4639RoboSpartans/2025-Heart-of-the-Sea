package frc.robot.subsystems.drive;

import choreo.trajectory.SwerveSample;
import com.ctre.phoenix6.SignalLogger;
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
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.limelight.LimelightHelpers;
import frc.lib.util.DriverStationUtil;
import frc.lib.util.PoseUtil;
import frc.robot.constants.Controls;
import frc.robot.constants.Limelights;
import frc.robot.subsystems.drive.constants.DriveConstants;
import frc.robot.subsystems.drive.constants.DrivePIDs;
import frc.robot.subsystems.drive.constants.TunerConstants;
import frc.robot.subsystems.drive.constants.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.subsystems.vision.Vision;

import java.util.Arrays;
import java.util.function.Supplier;

public class PhysicalSwerveDrivetrain extends AbstractSwerveDrivetrain {
    protected final TunerSwerveDrivetrain drivetrain;

    private boolean didApplyOperatorPerspective = false;

    private final PhoenixPIDController headingController = new PhoenixPIDController(28.48, 0, 1.1466);
    protected final PIDController
        pathXController = new PIDController(18, 0, 0),
        pathYController = new PIDController(18, 0, 0),
        pathHeadingController = new PIDController(7, 0, 0);
    protected ProfiledPIDController
        pidXController = constructPIDXController();
    protected ProfiledPIDController pidYController = constructPIDYController();

    public static ProfiledPIDController constructPIDYController() {
        return new ProfiledPIDController(3, 0, 0, new TrapezoidProfile.Constraints(2, 1));
    }

    public static ProfiledPIDController constructPIDXController() {
        return new ProfiledPIDController(3, 0, 0, new TrapezoidProfile.Constraints(2, 1));
    }

    {
        pathHeadingController.enableContinuousInput(-Math.PI, Math.PI);
        // Set up tunable numbers for drive pids
        DrivePIDs.pidToPoseXkP.onChange(pidXController::setP);
        DrivePIDs.pidToPoseYkP.onChange(pidYController::setP);
    }

    protected final Field2d field = new Field2d();

    private final SwerveSetpointGenerator swerveSetpointGenerator;
    private SwerveSetpoint prevSwerveSetpoint;
    private static final double MAX_STEER_VELOCITY_RADS_PER_SEC = 12.49;

    private boolean shouldUseMTSTDevs = false;

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

        RobotModeTriggers.autonomous().onTrue(new InstantCommand(() -> this.setVisionStandardDeviations(0.5, 0.5, 10)));
        RobotModeTriggers.teleop().onTrue(new InstantCommand(() -> this.setVisionStandardDeviations(10, 10, 1000)));
    }

    @Override
    public Command manualControl() {
        return applyRequest(this::getFieldCentricRequest);
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
    public Command pathfindTo(Pose2d targetPose) {
        double driveBaseRadius = 0;
        for (var moduleLocation : drivetrain.getModuleLocations()) {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        }
        PathConstraints constraints = new PathConstraints(
            1, 2,
            2 * Math.PI, 2 * Math.PI
        );
        return AutoBuilder.pathfindToPoseFlipped(
            targetPose,
            constraints
        );
    }

    @Override
    public Command _directlyMoveTo(Pose2d targetPose, Supplier<Pose2d> currentPose) {
        return new InstantCommand(() -> {
            pidXController.reset(getPose().getX(), getChassisSpeeds().vxMetersPerSecond);
            pidYController.reset(getPose().getY(), getChassisSpeeds().vyMetersPerSecond);
            pidXController.setGoal(targetPose.getX());
            pidYController.setGoal(targetPose.getY());
            SmartDashboard.putNumber("distanceThresholdMeters", 10);
            field.getObject("Target Pose").setPose(targetPose);
            SmartDashboard.putBoolean("Aligned", false);
            shouldUseMTSTDevs = true;
            Vision.addGlobalVisionMeasurements(this, shouldUseMTSTDevs);
        }).andThen(applyRequest(
                () -> {
                    pidXController.setConstraints(
                        new TrapezoidProfile.Constraints(1 * getSwerveSpeedMultiplier(), 1)
                    );
                    pidYController.setConstraints(
                        new TrapezoidProfile.Constraints(1 * getSwerveSpeedMultiplier(), 1)
                    );
                    SmartDashboard.putNumber("Distance to Target", PoseUtil.distanceBetween(targetPose, currentPose.get()));
                    field.getObject("Setpoint Pose").setPose(
                        new Pose2d(
                            new Translation2d(
                                pidXController.getSetpoint().position,
                                pidYController.getSetpoint().position
                            ),
                            targetPose.getRotation()
                        )
                    );
                    double directionMultiplier =
                        (DriverStationUtil.getAlliance() == DriverStation.Alliance.Red ? -1 : 1);
                    // if (RobotState.isAutonomous()) directionMultiplier = 1;
                    double pidXOutput = pidXController.calculate(currentPose.get().getX()) * directionMultiplier;
                    double pidYOutput = pidYController.calculate(currentPose.get().getY()) * directionMultiplier;

                    var request = new SwerveRequest.FieldCentricFacingAngle();
                    request.HeadingController = new PhoenixPIDController(16, 0, 0);
                    request.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
                    Rotation2d headingOffset = DriverStationUtil.getAlliance() == Alliance.Red ? Rotation2d.k180deg : new Rotation2d();

                    return request
                        .withVelocityX(pidXOutput)
                        .withVelocityY(pidYOutput)
                        .withTargetDirection(targetPose.getRotation().plus(headingOffset));
                }
            ).until(
                RobotState.isTeleop() ? () -> false : getAtTargetPoseTrigger(targetPose)
            )).andThen(stop().withTimeout(0.1))
            .finallyDo(() -> {
                setVisionStandardDeviations(5, 5, 10);
                SmartDashboard.putNumber("distanceThresholdMeters", 2);
                SmartDashboard.putBoolean("Aligned", true);
                shouldUseMTSTDevs = false;
            });
    }

    public Trigger getAtTargetPoseTrigger(Pose2d targetPose) {
        return new Trigger(() -> !RobotState.isTeleop() && atTargetPose(targetPose)).debounce(1);
    }

    public boolean atTargetPose(Pose2d targetPose) {
        return MathUtil.isNear(targetPose.getX(), getPose().getX(), 0.025)//0.01
            && MathUtil.isNear(targetPose.getY(), getPose().getY(), 0.025)//0.01
            && MathUtil.isNear(targetPose.getRotation().getDegrees(), getPose().getRotation().getDegrees(), 1);
    }

    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param requestSupplier Function returning the request to apply
     *
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
    public void resetPose(Pose2d pose) {
        drivetrain.resetPose(pose);
    }

    @Override
    public void addVisionMeasurement(Pose2d pose, double timestamp) {
        //field.getObject("Vision Measurement").setPose(pose);
        drivetrain.addVisionMeasurement(pose, timestamp);
    }

    @Override
    public void setVisionStandardDeviations(double xStdDev, double yStdDev, double rotStdDev) {
        drivetrain.setVisionMeasurementStdDevs(new Matrix<N3, N1>(Nat.N3(), Nat.N1(), new double[]{xStdDev, yStdDev, rotStdDev}));
    }
}