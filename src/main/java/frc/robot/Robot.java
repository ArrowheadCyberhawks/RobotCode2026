// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedPowerDistribution;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.vision.LimelightSubsystem;
import frc.robot.subsystems.shooter.*;

public class Robot extends LoggedRobot {
	private Command m_autonomousCommand;

	private final RobotContainer m_robotContainer;

	public Robot() {
		if (isReal()) {
			Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
			Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
		} else {
			setUseTiming(false); // Run as fast as possible
			String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
			Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
			Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
		}
		LoggedPowerDistribution.getInstance(32, ModuleType.kRev);
		Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.

		m_robotContainer = new RobotContainer();
	}

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
		m_robotContainer.updateField2d();
	}

	@Override
	public void disabledInit() {
		LimelightSubsystem.SetIMUMode(1);
		LimelightSubsystem.setThrottle(60);
	}

	@Override
	public void disabledPeriodic() {
	}

	@Override
	public void disabledExit() {
		LimelightSubsystem.setThrottle(0);
		LimelightSubsystem.SetIMUMode(4);
		m_robotContainer.questNavSubsystem.resetPose(m_robotContainer.drivetrain.getPose());
	}

	@Override
	public void autonomousInit() {
		m_autonomousCommand = m_robotContainer.getAutonomousCommand();

		if (m_autonomousCommand != null) {
			CommandScheduler.getInstance().schedule(m_autonomousCommand);
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
		if (m_autonomousCommand != null) {
			m_autonomousCommand.cancel();
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

	@Override
	public void simulationPeriodic() {
	}
}
