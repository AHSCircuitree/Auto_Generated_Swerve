// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;

public class RunIntake extends Command {
  /** Creates a new RunIntake. */

  Intake intake;
  Arm arm;
  double speed;

  public RunIntake(Intake Intake, Arm Arm, double Speed) {
    
    intake = Intake;
    arm = Arm;
    speed = Speed;

    addRequirements(Intake);
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.RunIntake(speed);
    arm.RunBottom(speed / 3);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.RunIntake(0);
    arm.RunBottom(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
