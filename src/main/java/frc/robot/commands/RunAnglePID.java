// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Limelight;

public class RunAnglePID extends Command {
  /** Creates a new RunAngle. */
  Arm m_arm;
  XboxController xbox;
 
  public RunAnglePID(Arm Arm, XboxController Xbox) {

    m_arm = Arm;
    xbox = Xbox;

    addRequirements(Arm );
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    // Climb is -110
    m_arm.ChangeTarget(70);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (xbox.getYButton() == true) {

      m_arm.ChangeTarget(-65);

    } 
    
    if (xbox.getBButton() == true) {

      m_arm.ChangeTarget(0);

    }
    
    if (xbox.getAButton() == true) {

      m_arm.ChangeTarget(70);

    } 

    if (xbox.getXButton() == true) {

      m_arm.ChangeTarget(SmartDashboard.getNumber("Custom Angle", 0));

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
