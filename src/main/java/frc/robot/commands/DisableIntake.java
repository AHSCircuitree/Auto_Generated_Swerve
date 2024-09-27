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

public class DisableIntake extends Command {
  /** Creates a new RunIntake. */

  Intake intake;
  Arm arm;
  Lights lights;
  Timer Spinup;

  public DisableIntake(Intake Intake, Arm Arm, Lights Lights) {
    
    intake = Intake;
    arm = Arm;
    lights = Lights;
    Spinup = new Timer();

    addRequirements(Intake);
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    intake.RunIntake(0);
    arm.RunBottom(0);
    lights.SetColor(Constants.Colors.Green);

  }

}