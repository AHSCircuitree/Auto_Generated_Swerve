// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.SetColor;
import frc.robot.commands.StatusCheck;
import frc.robot.commands.StatusCheck2;
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    
    CommandScheduler.getInstance().run(); 
   
    /* 
    var lastResult = LimelightHelpers.getLatestResults("limelight").targetingResults;

    Pose2d llPose = lastResult.getBotPose2d_wpiBlue();

    if (lastResult.valid) {

      m_robotContainer.drivetrain.addVisionMeasurement(llPose, Timer.getFPGATimestamp());
        
    }
    */
 
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {}

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
  public void testPeriodic() {

    CommandScheduler.getInstance().schedule(new StatusCheck2(m_robotContainer.drivetrain, m_robotContainer.hooks,
    m_robotContainer.arm, m_robotContainer.intake, m_robotContainer.lights, m_robotContainer.audio));

  }

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
