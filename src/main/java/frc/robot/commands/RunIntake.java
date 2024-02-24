// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lights;

public class RunIntake extends Command {
  /** Creates a new RunIntake. */

  Intake intake;
  Arm arm;
  Lights lights;
  double speed;
  Timer Spinup;

  public RunIntake(Intake Intake, Arm Arm, Lights Lights, double Speed) {
    
    intake = Intake;
    arm = Arm;
    speed = Speed;
    lights = Lights;
    Spinup = new Timer();

    addRequirements(Intake);
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    Spinup.restart();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
 
    intake.RunIntake(speed);
    arm.RunBottom(speed / 3);

    if (Spinup.get() > .5) {

      if (lights.GetState() == Constants.RobotState.RING_DETECTED && intake.CheckIntakeForPiece() == true) {

        lights.ChangeState(Constants.RobotState.RING_COLLECTED);

      }

    }
 
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
