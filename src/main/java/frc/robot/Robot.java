// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.VisionSubsystem;

public class Robot extends TimedRobot {
	private Command m_autonomousCommand;

	private final RobotContainer m_robotContainer;

	public Robot() {
		m_robotContainer = new RobotContainer();
	}

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
		m_robotContainer.periodic();
	}

	@Override
	public void disabledInit() {
		VisionSubsystem.SetIMUMode(1);
	}

	@Override
	public void disabledPeriodic() {
		VisionSubsystem.setThrottle(60);
		m_robotContainer.updateVisionPoseMT1();
	}

	@Override
	public void disabledExit() {
		VisionSubsystem.setThrottle(0);
		VisionSubsystem.SetIMUMode(2);
		m_robotContainer.questNav.resetPose(m_robotContainer.drivetrain.getPose());
	}

	@Override
	public void autonomousInit() {
		m_autonomousCommand = m_robotContainer.getAutonomousCommand();

		if (m_autonomousCommand != null) {
			m_autonomousCommand.schedule();
		}
	}

	@Override
	public void autonomousPeriodic() {}

	@Override
	public void autonomousExit() {}

	@Override
	public void teleopInit() {
		if (m_autonomousCommand != null) {
			m_autonomousCommand.cancel();
		}
	}

	@Override
	public void teleopPeriodic() {}

	@Override
	public void teleopExit() {}

	@Override
	public void testInit() {
		CommandScheduler.getInstance().cancelAll();
	}

	@Override
	public void testPeriodic() {}

	@Override
	public void testExit() {}

	@Override
	public void simulationPeriodic() {}
}
