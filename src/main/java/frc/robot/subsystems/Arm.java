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
import frc.robot.Constants;

public class Arm extends SubsystemBase {
 
  TalonFX AngleMotor;
  TalonFX BottomShootingMotor;
  TalonFX CentralShootingMotor;
  TalonFX TopShootingMotor;

  PIDController AnglePID;

  DutyCycleEncoder AngleEncoder;

  public double AngleVoltage;
  public double BottomShootingVoltage;
  public double CentralShootingVoltage;
  public double TopShootingVoltage;
  
  public Arm() {
 
    AngleMotor = new TalonFX(Constants.CAN_IDs.AngleID);
    BottomShootingMotor = new TalonFX(Constants.CAN_IDs.BottomShootingID);
    CentralShootingMotor = new TalonFX(Constants.CAN_IDs.CentralShootingID);
    TopShootingMotor = new TalonFX(Constants.CAN_IDs.TopShootingID);

    AnglePID = new PIDController(.3, 0, 0);

    AngleEncoder = new DutyCycleEncoder(6);
   
  }

  @Override
  public void periodic() {

    AngleVoltage = AngleMotor.getSupplyVoltage().getValueAsDouble();

    SmartDashboard.getNumber("Angle Voltage", AngleVoltage);
  
  }

  public void RunAngle(double speed) {

    AngleMotor.set(speed);
 
  }

  public void RunShooter(double speed) {

    BottomShootingMotor.set(speed);
    CentralShootingMotor.set(speed);
    TopShootingMotor.set(speed);

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
