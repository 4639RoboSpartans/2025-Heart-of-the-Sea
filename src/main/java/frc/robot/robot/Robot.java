// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.robot;

import au.grapplerobotics.CanBridge;
import com.ctre.phoenix6.SignalLogger;
import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.util.AllianceFlipUtil;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.scoring.ScoringSuperstructureAction;
import frc.robot.subsystems.scoring.constants.ScoringConstants;
import org.littletonrobotics.junction.LoggedRobot;


public class Robot extends LoggedRobot {
    private Command autonomousCommand;

    private final RobotContainer robotContainer;


    public Robot() {
        SignalLogger.enableAutoLogging(true);
        robotContainer = new RobotContainer();
        CanBridge.runTCP();
        SubsystemManager.getInstance().getDrivetrain().setVisionStandardDeviations(1, 1, 99999);
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
        SmartDashboard.putNumber("distanceThresholdMeters", 2);
    }


    @Override
    public void autonomousInit() {
        autonomousCommand = robotContainer.getAutonomousCommand().routine().get().cmd();
        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
        SmartDashboard.putNumber("distanceThresholdMeters", 100);
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
        SignalLogger.start();
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }


    @Override
    public void teleopPeriodic() {
    }


    @Override
    public void teleopExit() {
        SignalLogger.stop();
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
