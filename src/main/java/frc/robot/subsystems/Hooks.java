// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
 
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hooks extends SubsystemBase {

  TalonFX RightHookMotor;
  TalonFX LeftHookMotor;

  /** Creates a new Hooks. */
  public Hooks() {

    RightHookMotor = new TalonFX(29);
    LeftHookMotor = new TalonFX(30);
    // Use addRequirements() here to declare subsystem dependencies.
  }
 
  public void RunHooks(double speed) {

    RightHookMotor.set(speed);
    LeftHookMotor.set(speed);

  }

}
