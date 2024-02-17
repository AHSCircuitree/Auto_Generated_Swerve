// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Hooks;

public class RunHooksToAngle extends Command {
  /** Creates a new RunAngle. */
  Hooks hooks;
  XboxController xbox;
  double ClimbState;
 
  public RunHooksToAngle(Hooks Hooks, XboxController Xbox) {

    hooks = Hooks;
    xbox = Xbox;

    addRequirements(Hooks);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    hooks.SetTarget(-15);
    ClimbState = 1;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (ClimbState == 1 && xbox.getRightBumper() == true) {

      ClimbState = 2;

    }

    if (ClimbState == 2 && hooks.RightHookRelative > 200 && xbox.getRightBumper() == true) {

      ClimbState = 3;

    }

    if (ClimbState == 1) {

      hooks.SetTarget(-15);
      hooks.RunHooksToAngle();      

    } else if (ClimbState == 2) {

      hooks.RunHooksToRelative();

    } else if (ClimbState == 3) {

      hooks.SetTarget(-80);
      hooks.RunHooksToAngle();   

    } else {



    }
  
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    hooks.RunHooks(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
