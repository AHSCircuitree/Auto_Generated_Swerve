// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

  TalonFX LeftIntakeMotor;
  TalonFX RightIntakeMotor;
  TalonFX FrontIntakeMotor;
  TalonFX RearFlyMotor;
  TalonFX FrontFlyMotor;

  WPI_TalonSRX LeftIntake;

  public double LeftIntakeVoltage;
  public double RightIntakeVoltage;
  public double FrontIntakeVoltage;
  public double RearFlyVoltage;
  public double FrontFlyVoltage;

  /** Creates a new Intake. */
  public Intake() {

    LeftIntakeMotor = new TalonFX(Constants.CAN_IDs.LeftIntakeID, "FRC 1599");
    RightIntakeMotor = new TalonFX(Constants.CAN_IDs.RightIntakeID, "FRC 1599");
    FrontIntakeMotor = new TalonFX(Constants.CAN_IDs.FrontIntakeID, "FRC 1599");
    RearFlyMotor = new TalonFX(Constants.CAN_IDs.RearFlyID, "FRC 1599");
    FrontFlyMotor = new TalonFX(Constants.CAN_IDs.FrontFlyID, "FRC 1599");
 
  }

  @Override
  public void periodic() {

    LeftIntakeVoltage = LeftIntakeMotor.getSupplyVoltage().getValueAsDouble();
    RightIntakeVoltage = RightIntakeMotor.getSupplyVoltage().getValueAsDouble();
    FrontIntakeVoltage = FrontIntakeMotor.getSupplyVoltage().getValueAsDouble();
    FrontFlyVoltage = FrontFlyMotor.getSupplyVoltage().getValueAsDouble();
    RearFlyVoltage = RearFlyMotor.getSupplyVoltage().getValueAsDouble();

    SmartDashboard.putNumber("Left Intake Voltage", LeftIntakeVoltage);
    SmartDashboard.putNumber("Right Intake Voltage", RightIntakeVoltage);
    SmartDashboard.putNumber("Front Intake Voltage", FrontIntakeVoltage);
    SmartDashboard.putNumber("Front Fly Voltage", FrontFlyVoltage);
    SmartDashboard.putNumber("Back Fly Voltage", RearFlyVoltage);
    
  }

  public void RunIntake(double speed) {
    
    LeftIntakeMotor.set(-speed);
    //RightIntakeMotor.set(speed);
    FrontIntakeMotor.set(speed);
    FrontFlyMotor.set(speed * 10);
    RearFlyMotor.set(speed * 10);

  }

  public Boolean AreIntakeMotorsGood() {

    if (
    LeftIntakeVoltage < 9 ||
    RightIntakeVoltage < 9 ||
    FrontIntakeVoltage < 9 ||
    FrontFlyVoltage < 9 ||
    RearFlyVoltage < 9 ) {

      return false;

    } else {

      return true;

    }

  }

}
