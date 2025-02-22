// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class RunShooter extends Command {
  /** Creates a new RunAngle. */
  Arm arm;
  double speed;
  Timer topDelay;

  public RunShooter(Arm Arm, double Speed) {

    arm = Arm;
    speed = Speed;
    topDelay = new Timer();

    addRequirements(Arm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    topDelay.restart();

    speed = SmartDashboard.getNumber("Custom Speed", .5);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (topDelay.get() > .5) {

      arm.RunBottom(speed);

    }

    arm.RunShooter(speed);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    arm.RunShooter(0);
    arm.RunBottom(0);
    topDelay.stop();
    topDelay.reset();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
