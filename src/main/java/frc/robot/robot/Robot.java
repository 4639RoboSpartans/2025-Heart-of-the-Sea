// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.robot;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.scoring.ScoringSuperstructure;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;


public class Robot extends LoggedRobot {
    private Command autonomousCommand;

    private final RobotContainer robotContainer;


    public Robot() {
        SignalLogger.enableAutoLogging(true);
        robotContainer = new RobotContainer();
    }

    @Override
    public void robotInit() {
        Logger.start();
        ScoringSuperstructure.getInstance().setDefaultCommand(ScoringSuperstructure.getInstance().runScoringState());
    }


    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        Logger.recordOutput(
                "Robot Pose", new Pose2d()
        );
        Logger.recordOutput(
                "ZeroedComponentPoses",
                new Pose3d(),
                new Pose3d(),
                new Pose3d(),
                new Pose3d());
        Logger.recordOutput(
                "FinalComponentPoses",
                new Pose3d(),
                new Pose3d(),
                new Pose3d(),
                new Pose3d());
    }


    @Override
    public void disabledInit() {

    }


    @Override
    public void disabledPeriodic() {
    }


    @Override
    public void disabledExit() {
    }


    @Override
    public void autonomousInit() {
        autonomousCommand = robotContainer.getAutonomousCommand();

        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
    }


    @Override
    public void autonomousPeriodic() {
    }


    @Override
    public void autonomousExit() {
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
