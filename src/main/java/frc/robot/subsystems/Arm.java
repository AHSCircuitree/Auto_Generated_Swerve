// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
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

  public double CurrentTicks;
  public double CurrentAngle;
  public double TargetAngle;

  public double AngleVoltage;
  public double BottomShootingVoltage;
  public double CentralShootingVoltage;
  public double TopShootingVoltage;
 
  public double CustomArmSpeed;
 
  public Arm() {
 
    AngleMotor = new TalonFX(Constants.CAN_IDs.AngleID,"FRC 1599");
    BottomShootingMotor = new TalonFX(Constants.CAN_IDs.BottomShootingID,"FRC 1599");
    CentralShootingMotor = new TalonFX(Constants.CAN_IDs.CentralShootingID,"FRC 1599");
    TopShootingMotor = new TalonFX(Constants.CAN_IDs.TopShootingID,"FRC 1599");

    AngleMotor.setNeutralMode(NeutralModeValue.Brake);

    AnglePID = new PIDController(.03, 0, 0);

    AngleEncoder = new DutyCycleEncoder(6);

    AngleEncoder.setPositionOffset(.828);
 
    SmartDashboard.putNumber("Custom Angle", 0);
    SmartDashboard.putNumber("Custom Speed",0);
 
  }

  @Override
  public void periodic() {

    AngleVoltage = AngleMotor.getSupplyVoltage().getValueAsDouble();
 
    if (AngleEncoder.getAbsolutePosition() < .2) {

      CurrentTicks = AngleEncoder.getAbsolutePosition() + 1;
      
    } else {

      CurrentTicks = AngleEncoder.getAbsolutePosition();

    }

    CurrentAngle = -(CurrentTicks / (.072 / 28) - 328) - 14;

    SmartDashboard.getNumber("Angle Voltage", AngleVoltage);
    SmartDashboard.putNumber("Angle Encoder Ticks", AngleEncoder.getAbsolutePosition());
    SmartDashboard.putNumber("Angle Encoder Degrees", CurrentAngle);
    SmartDashboard.putNumber("Arm Speed", AnglePID.calculate(CurrentAngle, TargetAngle));

  }

  public void RunAngle(double speed) {

    AngleMotor.set(speed);
 
  }

  public void RunAngleWithLimits(double speed) {

    AngleMotor.set(speed);
 
  }

  public void ChangeAngleThroughPID() {

    RunAngleWithLimits(AnglePID.calculate(CurrentAngle, TargetAngle) );
 
  }

  public void ChangeTarget(double Target) {

    TargetAngle = Target;

  }

  public void RunShooter(double speed) {
 
    BottomShootingMotor.set(-speed);
    CentralShootingMotor.set(-speed);
    TopShootingMotor.set(-speed);

  }

  public void Spinup(double speed) {

    TopShootingMotor.set(-speed);
    CentralShootingMotor.set(-speed);
  }

  public void SpinupAuto(double speed) {
 
    CentralShootingMotor.set(-speed);
  }

  public void RunBottom(double speed) {

    BottomShootingMotor.set(-speed);

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
