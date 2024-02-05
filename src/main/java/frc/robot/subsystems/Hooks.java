// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hooks extends SubsystemBase {

  TalonFX RightHookMotor;
  TalonFX LeftHookMotor;
  PIDController HookPID;
  DutyCycleEncoder HookEncoder;


  public double RightHookVoltage;
  public double LeftHookVoltage;

  /** Creates a new Hooks. */
  public Hooks() {

    RightHookMotor = new TalonFX(29);
    LeftHookMotor = new TalonFX(30);
    HookPID = new PIDController(.3, 0, 0);
    //
    HookEncoder = new DutyCycleEncoder(0);
    // Use addRequirements() here to declare subsystem dependencies.
  }
  public void RunHooks(double speed) {

    RightHookMotor.set(speed);
    LeftHookMotor.set(speed);

  }

  public void HookPID(double Setpoint){
    RightHookMotor.set(HookPID.calculate(HookEncoder.getAbsolutePosition(), Setpoint));
    LeftHookMotor.set(HookPID.calculate(HookEncoder.getAbsolutePosition(), Setpoint));
  }
   
  public void periodic(){
    RightHookVoltage = RightHookMotor.getSupplyVoltage().getValueAsDouble();
    LeftHookVoltage = LeftHookMotor.getSupplyVoltage().getValueAsDouble();
    
    SmartDashboard.getNumber("Left Hook Voltage", LeftHookVoltage);
    SmartDashboard.getNumber("Right Hook Voltage", RightHookVoltage);
  }

  public boolean AreHookMotorsGood(){
    
    if (
      LeftHookVoltage < 9 ||
      RightHookVoltage < 9) {

        return false;
     
      } else {
        
        return true;

      }
   }
}
