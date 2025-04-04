// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.robot;

import au.grapplerobotics.CanBridge;
import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.util.AllianceFlipUtil;
import frc.lib.util.DriverStationUtil;
import frc.robot.commands.auto.AutoRoutines;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.scoring.ScoringSuperstructureAction;
import frc.robot.subsystems.scoring.constants.ScoringConstants;
import org.littletonrobotics.junction.LoggedRobot;


public class Robot extends LoggedRobot {
    private Command autonomousCommand;

    private final RobotContainer robotContainer;
    private final Field2d startingPoseField = new Field2d();


    public Robot() {
        robotContainer = new RobotContainer();
        CanBridge.runTCP();
        SubsystemManager.getInstance().getDrivetrain().setVisionStandardDeviations(0.5, 0.5, 0.1);
    }

    @Override
    public void robotInit() {
        SubsystemManager.getInstance().instantiateSubsystems();
        ScoringConstants.EndEffectorConstants.RotationStartingPosition = SubsystemManager.getInstance().getScoringSuperstructure().getEndEffectorSubsystem().getCurrentRotation();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        robotContainer.add3DComponentPoses();
        SmartDashboard.putBoolean("DS Alliance", AllianceFlipUtil.shouldFlip());
        SmartDashboard.putNumber("Match Time", Timer.getMatchTime());
        SmartDashboard.putBoolean("DS connected", DriverStationUtil.hasDSAlliance());
    }

    @Override
    public void disabledInit() {
        SubsystemManager.getInstance().getScoringSuperstructure().setAction(ScoringSuperstructureAction.IDLE);
        SubsystemManager.getInstance().getScoringSuperstructure().getEndEffectorSubsystem().setWristMotorIdleMode(SparkBaseConfig.IdleMode.kCoast);
        SmartDashboard.putNumber("distanceThresholdMeters", 100);
    }


    @Override
    public void disabledPeriodic() {
    }


    @Override
    public void disabledExit() {
        SubsystemManager.getInstance().getScoringSuperstructure().getEndEffectorSubsystem().setWristMotorIdleMode(SparkBaseConfig.IdleMode.kBrake);
    }


    @Override
    public void autonomousInit() {
        AutoRoutines.Auton autonSupplier = robotContainer.getSelectedAuton();
        if (autonSupplier != null) {
            Pose2d startingPose = autonSupplier.startPose();
            autonomousCommand = autonSupplier.routine().cmd();
            startingPoseField.setRobotPose(startingPose);
            SmartDashboard.putData("Starting Pose", startingPoseField);
        }
        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {
    }


    @Override
    public void autonomousExit() {
        SmartDashboard.putNumber("distanceThresholdMeters", 2);
    }


    @Override
    public void teleopInit() {
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }


    @Override
    public void teleopPeriodic() {
    }


    @Override
    public void teleopExit() {
    }


    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }


    @Override
    public void testPeriodic() {
    }


    @Override
    public void testExit() {
    }
}
