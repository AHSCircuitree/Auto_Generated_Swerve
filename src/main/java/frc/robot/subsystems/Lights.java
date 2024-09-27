// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Lights extends SubsystemBase {
  /** Creates a new LightSubsystem. */

  Spark blinkin; 

  private Constants.RobotState CurrentState = Constants.RobotState.NO_RING;

  public Lights() {

    blinkin = new Spark(1);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void SetColor(double color) {

    blinkin.set(color);

  }

  public void ChangeColorOffRobotState() {

    if (CurrentState == Constants.RobotState.NO_RING) {

      blinkin.set(Constants.Colors.Red);

    } else if (CurrentState == Constants.RobotState.RING_DETECTED) {

      blinkin.set(Constants.Colors.Orange);

    } else if (CurrentState == Constants.RobotState.RING_COLLECTED) {

      blinkin.set(Constants.Colors.Blue);

    } else {

      blinkin.set(Constants.Colors.Green);

    }

  }

  public void ChangeState(Constants.RobotState State) {

    CurrentState = State;

  }

  public Constants.RobotState GetState() {

    return CurrentState;

  }

}

