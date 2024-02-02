// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Limelight;

public class RumbleOnTarget extends Command {

  private Limelight limelight;
  private Lights lights;
  private XboxController xbox;
 
  public RumbleOnTarget(Limelight Limelight, Lights Lights, XboxController Xbox) {

    limelight = Limelight;
    lights = Lights;
    xbox = Xbox;

    addRequirements(Limelight, Lights);
 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    limelight.updateLimelightTracking();
    
    if (limelight.HorizonalOffset_RI() != 0) {
 
      xbox.setRumble(RumbleType.kLeftRumble, 0.5);
      lights.SetColor(.73);
       
    } else {

      xbox.setRumble(RumbleType.kLeftRumble, 0.0);
      lights.SetColor(.61);

    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
