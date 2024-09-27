// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Limelight;
 
public class ChangeLightsBasedOffState extends Command {
  /** Creates a new ChangeLightsBasedOffState. */
  Lights m_lights;
  Limelight m_limelight;
  XboxController m_xbox;
  
  public ChangeLightsBasedOffState(Lights lights, Limelight limelight, XboxController xbox) {

    m_limelight = limelight;
    m_lights = lights;
    m_xbox = xbox;

    addRequirements(lights);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if ((Math.abs(m_limelight.dbl_tx_li) < 1 || Math.abs(m_limelight.dbl_tx_ri) < 1) && m_lights.GetState() == Constants.RobotState.NO_RING) {

      m_lights.ChangeState(Constants.RobotState.RING_DETECTED);

    } else if ((Math.abs(m_limelight.dbl_tx_li) < 1 && Math.abs(m_limelight.dbl_tx_ri) < 1) && m_lights.GetState() == Constants.RobotState.RING_DETECTED) {

      m_lights.ChangeState(Constants.RobotState.NO_RING);

    } 

    if ((m_limelight.dbl_tx != 0 && m_lights.GetState() == Constants.RobotState.RING_COLLECTED)) {

      m_lights.ChangeState(Constants.RobotState.VALID_TARGET);

    }

    m_lights.ChangeColorOffRobotState();

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
