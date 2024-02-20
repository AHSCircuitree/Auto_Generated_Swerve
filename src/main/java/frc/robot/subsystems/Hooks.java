// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Hooks extends SubsystemBase {

  //Line
  TalonFX RightHookMotor;
  TalonFX LeftHookMotor;
  PIDController HookPID;
 
  DutyCycleEncoder LeftHookEncoder;
  DutyCycleEncoder RightHookEncoder;
 
  public double RightHookVoltage;
  public double LeftHookVoltage;

  public double RightHookDegrees;
  public double RightHookRelative;

  public double TargetAngle;

  /** Creates a new Hooks. */
  public Hooks() {

    RightHookMotor = new TalonFX(Constants.CAN_IDs.RightHookID,"FRC 1599");
    LeftHookMotor = new TalonFX(Constants.CAN_IDs.LeftHookID,"FRC 1599");

    RightHookMotor.setNeutralMode(NeutralModeValue.Brake);
    LeftHookMotor.setNeutralMode(NeutralModeValue.Brake);

    HookPID = new PIDController(.3, 0, 0);
    
    LeftHookEncoder = new DutyCycleEncoder(1);
    RightHookEncoder = new DutyCycleEncoder(2);

    TargetAngle = -15;
 
 
  }
  public void RunHooks(double speed) {

    RightHookMotor.set(speed);
    LeftHookMotor.set(-speed);

  }

  public void HookPID(double Setpoint) {
 
    RightHookMotor.set(HookPID.calculate(LeftHookEncoder.getAbsolutePosition(), Setpoint));
    LeftHookMotor.set(HookPID.calculate(LeftHookEncoder.getAbsolutePosition(), Setpoint));
 
  }
   
  public void periodic() {

    RightHookVoltage = RightHookMotor.getSupplyVoltage().getValueAsDouble();
    LeftHookVoltage = LeftHookMotor.getSupplyVoltage().getValueAsDouble();
    
    SmartDashboard.getNumber("Left Hook Voltage", LeftHookVoltage);
    SmartDashboard.getNumber("Right Hook Voltage", RightHookVoltage);

    SmartDashboard.putNumber("Left Hook Absolute", LeftHookEncoder.getAbsolutePosition());
    SmartDashboard.putNumber("Right Hook Absolute", RightHookEncoder.getAbsolutePosition());

    SmartDashboard.putNumber("Left Hook Relative", LeftHookMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Right Hook Relative", RightHookMotor.getPosition().getValueAsDouble());

    SmartDashboard.putNumber("Right Hook Degrees", -(RightHookEncoder.getAbsolutePosition() - 0.471) / (.0256 / 9));

    RightHookDegrees = -(RightHookEncoder.getAbsolutePosition() - 0.471) / (.0256 / 9);
    RightHookRelative = RightHookMotor.getPosition().getValueAsDouble();

  }

  public void SetTarget(double Target) {

    TargetAngle = Target;

  }

  public void RunHooksToAngle() {

    if (RightHookDegrees < TargetAngle && Math.abs(RightHookDegrees - TargetAngle) > 3) {

      RunHooks(.4);

    } else if (RightHookDegrees > TargetAngle && Math.abs(RightHookDegrees - TargetAngle) > 3) {

      RunHooks(-.4);

    } else {

      RunHooks(0);

    }

  }

  public void RunHooksToRelative() {

    double Relative = 260;
    if (RightHookRelative < Relative) {

      RunHooks(.4);

    } else {

      RunHooks(0);

    }

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
