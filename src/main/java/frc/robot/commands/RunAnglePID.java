// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class RunAnglePID extends Command {
  /** Creates a new RunAngle. */
  Arm arm;
  XboxController xbox;
 
  public RunAnglePID(Arm Arm, XboxController Xbox) {

    arm = Arm;
    xbox = Xbox;

    addRequirements(Arm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    arm.ChangeTarget(65);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (xbox.getYButton() == true) {

      arm.ChangeTarget(-55);

    } 
    
    if (xbox.getBButton() == true) {

      arm.ChangeTarget(0);

    }
    
    if (xbox.getAButton() == true) {

      arm.ChangeTarget(65);

    } 

    if (xbox.getXButton() == true) {

      arm.ChangeTarget(SmartDashboard.getNumber("Custom Angle", 0));

    } 

    arm.ChangeAngleThroughPID();
 
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    arm.RunAngle(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
