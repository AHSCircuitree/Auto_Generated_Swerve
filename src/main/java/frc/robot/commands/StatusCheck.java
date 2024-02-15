// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Audio;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Hooks;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lights; 

public class StatusCheck extends InstantCommand {

  Audio audio;
  Arm arm;
  Hooks hooks;
  Drivetrain drivetrain;
  Intake intake;
  Lights lights;

  public StatusCheck(Drivetrain Drivetrain, Hooks Hooks, Arm Arm, Intake Intake, Lights Lights, Audio Audio) {

    arm = Arm;
    drivetrain = Drivetrain;
    hooks = Hooks;
    intake = Intake;
    lights = Lights;
    audio = Audio;

    addRequirements(Drivetrain, Lights);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    if (drivetrain.IsDriveTrainGood() == false) {

      lights.SetColor(Constants.Colors.Red);
 
    } else {

      lights.SetColor(Constants.Colors.Green);

    }


  }
}
