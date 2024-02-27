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

public class RunAnglePID extends Command {
  /** Creates a new RunAngle. */
  Arm m_arm;
  Limelight m_limelight;
  Hooks m_hooks;
  Lights m_lights;
  XboxController xbox1;
  XboxController xbox2;

  public enum ToggleLimelight {

    TOGGLE_ON,
    TOGGLE_OFF

  }

  public ToggleLimelight TrackingStatus = ToggleLimelight.TOGGLE_OFF;
  public boolean HasGoneBackToIntake = false;
 
  public RunAnglePID(Arm Arm, Hooks Hooks, XboxController Xbox1, XboxController Xbox2, Limelight Limelight, Lights Lights) {

    m_arm = Arm;
    m_hooks = Hooks;
    m_lights = Lights;
    xbox1 = Xbox1;
    xbox2 = Xbox2;
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

    if (xbox1.getXButton() == true) {

      m_arm.ChangeTarget(-65);
      TrackingStatus = ToggleLimelight.TOGGLE_OFF;

    } 

    if (xbox2.getXButton() == true) {

      m_arm.ChangeTarget(-65);
      TrackingStatus = ToggleLimelight.TOGGLE_OFF;

    } 
 
    if (xbox1.getBButton() == true) {

      m_arm.ChangeTarget(m_arm.CurrentAngle);
      TrackingStatus = ToggleLimelight.TOGGLE_OFF;

    }

    if (xbox2.getBButton() == true) {

      m_arm.ChangeTarget(m_arm.CurrentAngle);
      TrackingStatus = ToggleLimelight.TOGGLE_OFF;

    }
    
    if (xbox1.getAButton() == true) {

      m_arm.ChangeTarget(55);
      TrackingStatus = ToggleLimelight.TOGGLE_OFF;

    } 

    if (xbox2.getYButton() == true) {

      m_arm.ChangeTarget(-117);
      TrackingStatus = ToggleLimelight.TOGGLE_OFF;

    }

    // turns on limelight adjustment
    if (xbox1.getRightBumper() == true) {

     TrackingStatus = ToggleLimelight.TOGGLE_ON;

    }

    // Left DPad
    if (xbox1.getPOV() < 280 && xbox1.getPOV() > 260) {

     TrackingStatus = ToggleLimelight.TOGGLE_OFF;

    }

    if (TrackingStatus == ToggleLimelight.TOGGLE_ON) {

      if (m_limelight.dbl_tx != 0) {

        m_lights.SetColor(Constants.Colors.Green);

      } else {

        m_lights.SetColor(Constants.Colors.Yellow);

      } 

      int distance = (int) Math.round(m_limelight.getDistanceToAprilTag() * 10);
      m_arm.ChangeTarget(Constants.ShootAngle[distance]);
      SmartDashboard.putNumber("Expected Angle", Constants.ShootAngle[distance]);

      HasGoneBackToIntake = true;

    }

    if (TrackingStatus == ToggleLimelight.TOGGLE_OFF) {

      if (m_limelight.dbl_tx != 0) {

        m_lights.SetColor(Constants.Colors.BlueGreen);

      } else {

        m_lights.SetColor(Constants.Colors.HotPink);

      }
    

      if (HasGoneBackToIntake == true) {

        m_arm.ChangeTarget(55);
        HasGoneBackToIntake = false;

      }

    }

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
