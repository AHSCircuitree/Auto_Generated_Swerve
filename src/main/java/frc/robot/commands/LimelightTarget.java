// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Hooks;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Limelight;

public class LimelightTarget extends Command {
  /** Creates a new RunAngle. */
  Arm m_arm;
  Limelight m_limelight;
  Hooks m_hooks;
  Lights m_lights;
  XboxController xbox;

  public enum ToggleLimelight {

    TOGGLE_ON,
    TOGGLE_OFF

  }

  public ToggleLimelight TrackingStatus = ToggleLimelight.TOGGLE_OFF;
  public boolean HasGoneBackToIntake = false;
 
  public LimelightTarget(Arm Arm, Hooks Hooks, XboxController Xbox, Limelight Limelight, Lights Lights) {

    m_arm = Arm;
    m_hooks = Hooks;
    m_lights = Lights;
    xbox = Xbox;
    m_limelight = Limelight;

    addRequirements(Arm, Lights);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    TrackingStatus = ToggleLimelight.TOGGLE_OFF;
    m_arm.ChangeTarget(m_arm.CurrentAngle);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
 
    
    m_lights.SetColor(Constants.Colors.Green);
    int distance = (int) Math.round(m_limelight.getDistanceToAprilTag() * 10);
    m_arm.ChangeTarget(Constants.ShootAngle[distance]);
    SmartDashboard.putNumber("Expected Angle", Constants.ShootAngle[distance]);
 
    m_arm.ChangeAngleThroughPID();
 
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_arm.RunAngle(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
