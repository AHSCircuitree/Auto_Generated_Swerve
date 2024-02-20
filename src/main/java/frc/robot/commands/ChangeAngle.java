// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class ChangeAngle extends Command {
  /** Creates a new RunAngle. */
  Arm arm;
  double target;
 
  public ChangeAngle(Arm Arm, double Target) {

    arm = Arm;
    target = Target;
 
    addRequirements(Arm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    arm.ChangeTarget(target);

  }
 
}
