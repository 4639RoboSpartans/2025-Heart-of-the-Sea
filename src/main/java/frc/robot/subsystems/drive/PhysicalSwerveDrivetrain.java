package frc.robot.subsystems.drive;

import choreo.trajectory.SwerveSample;
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
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.util.DriverStationUtil;
import frc.robot.constants.Controls;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.drive.constants.DriveConstants;
import frc.robot.subsystems.drive.constants.DrivePIDs;
import frc.robot.subsystems.drive.constants.TunerConstants;
import frc.robot.subsystems.drive.constants.TunerConstants.TunerSwerveDrivetrain;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

public class PhysicalSwerveDrivetrain extends AbstractSwerveDrivetrain {
    protected final TunerSwerveDrivetrain drivetrain;

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
    }

    @Override
    public Command manualControl() {
        return applyRequest(this::getFieldCentricRequest);
    }

    private SwerveRequest getFieldCentricRequest() {
        double rawForwards = Controls.Driver.SwerveForwardAxis.getAsDouble() * DriveConstants.CURRENT_MAX_ROBOT_MPS;
        double rawStrafe = -Controls.Driver.SwerveStrafeAxis.getAsDouble() * DriveConstants.CURRENT_MAX_ROBOT_MPS;
        double rawRotation = Controls.Driver.SwerveRotationAxis.getAsDouble() * DriveConstants.TELOP_ROTATION_SPEED;
        double allianceBasedDirection = DriverStationUtil.getAlliance() == Alliance.Blue ? 1 : -1;
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

    @Override
    public Command resetPigeon() {
        return runOnce(
            () -> drivetrain.getPigeon2().setYaw(
                Angle.ofBaseUnits(
                    0,
                    Degrees
                )
            )
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
            10, 5,
            2 * Math.PI, 2 * Math.PI
        );
        return AutoBuilder.pathfindToPoseFlipped(
            targetPose,
            constraints
        );
    }

    @Override
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
        SubsystemManager.getInstance().getVisionSubsystem().getVisionResults().forEach(
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
}