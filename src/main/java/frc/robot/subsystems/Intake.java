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

  TalonFX LeftIntakeMotor;
  TalonFX RightIntakeMotor;
  TalonFX FrontIntakeMotor;
  TalonFX RearFlyMotor;
  TalonFX FrontFlyMotor;

  public double LeftIntakeVoltage;
  public double RightIntakeVoltage;
  public double FrontIntakeVoltage;
  public double RearFlyVoltage;
  public double FrontFlyVoltage;

  /** Creates a new Intake. */
  public Intake() {

    LeftIntakeMotor = new TalonFX(Constants.CAN_IDs.LeftIntakeID);
    RightIntakeMotor = new TalonFX(Constants.CAN_IDs.RightIntakeID);
    FrontIntakeMotor = new TalonFX(Constants.CAN_IDs.FrontIntakeID);
    RearFlyMotor = new TalonFX(Constants.CAN_IDs.RearFlyID);
    FrontFlyMotor = new TalonFX(Constants.CAN_IDs.FrontFlyID);

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

  public void RunIntakeVelocity(double speed) {

    VelocityVoltage velocity = new VelocityVoltage(0);

    var slot0Configs = new Slot0Configs();
    slot0Configs.kV = 0.12;
    slot0Configs.kP = 0.11;
    slot0Configs.kI = 0.48;
    slot0Configs.kD = 0.01;

    LeftIntakeMotor.getConfigurator().apply(slot0Configs, 0.050);
    RightIntakeMotor.getConfigurator().apply(slot0Configs, 0.050);
    FrontIntakeMotor.getConfigurator().apply(slot0Configs, 0.050);
    FrontFlyMotor.getConfigurator().apply(slot0Configs, 0.050);
    RearFlyMotor.getConfigurator().apply(slot0Configs, 0.050);

    // periodic, run velocity control with slot 0 configs,
    // target velocity of 50 rps
    velocity.Slot = 0;
    
    LeftIntakeMotor.setControl(velocity.withFeedForward(speed));
    RightIntakeMotor.setControl(velocity.withFeedForward(speed));
    FrontIntakeMotor.setControl(velocity.withFeedForward(speed));
    FrontFlyMotor.setControl(velocity.withFeedForward(speed));
    RearFlyMotor.setControl(velocity.withFeedForward(speed));

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
