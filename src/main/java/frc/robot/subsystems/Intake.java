// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
 
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
 
  TalonFX FrontIntakeMotor;
  TalonFX RearFlyMotor;
  TalonFX FrontFlyMotor;
  
  public double FrontIntakeVoltage;
  public double RearFlyVoltage;
  public double FrontFlyVoltage;

  /** Creates a new Intake. */
  public Intake() {
  
    FrontIntakeMotor = new TalonFX(Constants.CAN_IDs.FrontIntakeID, "FRC 1599");
    RearFlyMotor = new TalonFX(Constants.CAN_IDs.RearFlyID, "FRC 1599");
    FrontFlyMotor = new TalonFX(Constants.CAN_IDs.FrontFlyID, "FRC 1599");
 
  }

  @Override
  public void periodic() {
 
    FrontIntakeVoltage = FrontIntakeMotor.getSupplyVoltage().getValueAsDouble();
    FrontFlyVoltage = FrontFlyMotor.getSupplyVoltage().getValueAsDouble();
    RearFlyVoltage = RearFlyMotor.getSupplyVoltage().getValueAsDouble();
 
    SmartDashboard.putNumber("Front Intake Voltage", FrontIntakeVoltage);
    SmartDashboard.putNumber("Front Fly Voltage", FrontFlyVoltage);
    SmartDashboard.putNumber("Back Fly Voltage", RearFlyVoltage);
    
  }

  public void RunIntake(double speed) {
 
    FrontIntakeMotor.set(speed);
    FrontFlyMotor.set(speed * 10);
    RearFlyMotor.set(speed * 10);
 
  }

  public boolean CheckIntakeForPiece() {

    if (FrontFlyMotor.getMotorVoltage().getValueAsDouble() < 8) {

      return true;

    } else {

      return false;

    }

  }

  public Boolean AreIntakeMotorsGood() {

    if (
    
    FrontIntakeVoltage < 9 ||
    FrontFlyVoltage < 9 ||
    RearFlyVoltage < 9 ) {

      return false;

    } else {

      return true;

    }

  }

}
