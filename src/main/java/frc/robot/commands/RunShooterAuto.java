// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Lights;

public class RunShooterAuto extends Command {
  /** Creates a new RunAngle. */
  Arm arm;
  Lights lights;
  double speed;
  Timer Spinup;

  public RunShooterAuto(Arm Arm, Lights Lights, double Speed) {

    arm = Arm;
    speed = Speed;
    lights = Lights;
    Spinup = new Timer();

    addRequirements(Arm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    Spinup.restart();

   // speed = SmartDashboard.getNumber("Custom Speed", .5);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (Spinup.get() < 0.1) {

      arm.RunBottom(-.05);

    } else if (Spinup.get() < 0.4) {

      arm.RunShooter(speed);

    } 

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    lights.ChangeState(Constants.RobotState.NO_RING);
    arm.RunBottom(0);
    arm.SpinupAuto(0);
    Spinup.stop();
    Spinup.reset();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
