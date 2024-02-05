// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  CANSparkMax LeftIntakeMotor;
  CANSparkMax RightIntakeMotor;
  CANSparkMax FrontIntakeMotor;
  TalonFX RearFlyMotor;
  TalonFX FrontFlyMotor;
  /** Creates a new Intake. */
  public Intake() {

    LeftIntakeMotor = new CANSparkMax(21, MotorType.kBrushless);
    RightIntakeMotor = new CANSparkMax(22, MotorType.kBrushless);
    FrontIntakeMotor = new CANSparkMax(23, MotorType.kBrushless);
    RearFlyMotor = new TalonFX(24);
    FrontFlyMotor = new TalonFX(25);

    RightIntakeMotor.setInverted(true);

  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("Left Intake Voltage", LeftIntakeMotor.getBusVoltage());
    SmartDashboard.putNumber("Right Intake Voltage", RightIntakeMotor.getBusVoltage());
    SmartDashboard.putNumber("Front Intake Voltage", FrontIntakeMotor.getBusVoltage());
    SmartDashboard.putNumber("Front Fly Voltage", FrontFlyMotor.getSupplyVoltage().getValueAsDouble());
    SmartDashboard.putNumber("Back Fly Voltage", RearFlyMotor.getSupplyVoltage().getValueAsDouble());
    
  }

  public void RunIntake(double speed) {
    
      LeftIntakeMotor.set(speed);
      RightIntakeMotor.set(speed);
      FrontIntakeMotor.set(speed);
      FrontFlyMotor.set(speed);
      RearFlyMotor.set(speed);

  }

}
