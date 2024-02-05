// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
 
  TalonFX AngleMotor;
  PIDController AnglePID;
  DutyCycleEncoder AngleEncoder;

  public double AngleVoltage;
  
  public Arm() {
 
    AngleMotor = new TalonFX(31);
    AnglePID = new PIDController(.3, 0, 0);
    //AngleEncoder = new DutyCycleEncoder(0);
   
  }

  @Override
  public void periodic() {
   AngleVoltage = AngleMotor.getSupplyVoltage().getValueAsDouble();

   SmartDashboard.getNumber("Angle Voltage", AngleVoltage);
 
    // This method will be called once per scheduler run
  }

  public void RunAngle(double speed) {

    AngleMotor.set(speed);
 
  }

  public void AnglePID(double Setpoint) {

    AngleMotor.set(AnglePID.calculate(AngleEncoder.getAbsolutePosition(), Setpoint));
 
  }
  
  public boolean AreAngleMotorsGood(){

    if (
      AngleVoltage < 9) {

        return false;

      } else {

        return true;

      }
  }
  
}
