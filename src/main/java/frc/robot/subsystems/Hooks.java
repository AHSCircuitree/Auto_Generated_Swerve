// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
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
 
  public Timer RotationalVelocityRefresh;
  public double RotationalVelocityRefreshTime = .4;
  public double PreviousRotationPosition = 10;
  public double RotationalVelocity = 0;
  public enum HookState {

    UNKNOWN,
    ON_ANGLE,
    STRAIGHT,
    CALIBRATED

  }
  public HookState CurrentState = HookState.UNKNOWN;
  public String HookMessage = "Unknown State: Please move the hook";

  /** Creates a new Hooks. */
  public Hooks() {
 
    RightHookMotor = new TalonFX(Constants.CAN_IDs.RightHookID,"FRC 1599");
    LeftHookMotor = new TalonFX(Constants.CAN_IDs.LeftHookID,"FRC 1599");

    RightHookMotor.setNeutralMode(NeutralModeValue.Brake);
    LeftHookMotor.setNeutralMode(NeutralModeValue.Brake);

    HookPID = new PIDController(.3, 0, 0);
    
    LeftHookEncoder = new DutyCycleEncoder(1);
    RightHookEncoder = new DutyCycleEncoder(2);
 
    RotationalVelocityRefresh = new Timer();

    RotationalVelocityRefresh.start();
 
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

    SmartDashboard.putNumber("Left Hook Absolute", -LeftHookEncoder.getAbsolutePosition());
    SmartDashboard.putNumber("Right Hook Absolute", -RightHookEncoder.getAbsolutePosition());

    SmartDashboard.putNumber("Left Hook Relative", LeftHookMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Right Hook Relative", RightHookMotor.getPosition().getValueAsDouble());
 
    if (RotationalVelocityRefresh.get() > RotationalVelocityRefreshTime) {

      RotationalVelocity = Math.abs((LeftHookEncoder.getAbsolutePosition() / (.072 / 28)) - PreviousRotationPosition);
      PreviousRotationPosition = Math.abs((LeftHookEncoder.getAbsolutePosition() / (.072 / 28)));
      RotationalVelocityRefresh.restart();

    }

    SmartDashboard.putNumber("Rotational Velocity", RotationalVelocity);
    SmartDashboard.putNumber("Hook Relative", LeftHookMotor.getPosition().getValueAsDouble());
    SmartDashboard.putString("Hook State", HookMessage);

  }
  
  public void UpdateHookState() {

    if (RotationalVelocity > 3 && CurrentState != HookState.CALIBRATED) {

      if (CurrentState == HookState.STRAIGHT) {

        LeftHookMotor.setPosition(0);
        HookMessage = "Calibrated: Should function as expected";
        CurrentState = HookState.CALIBRATED;

      } else {

        CurrentState = HookState.ON_ANGLE;
        LeftHookMotor.setPosition(0);
        HookMessage = "Angled: Please move the hook up to calibrate";

      }

    }

    if (RotationalVelocity < 3 && CurrentState != HookState.CALIBRATED) {
 
      if (CurrentState == HookState.ON_ANGLE) {

        LeftHookMotor.setPosition(0);
        HookMessage = "Calibrated: Should function as expected";
        CurrentState = HookState.CALIBRATED;

      } else {

        CurrentState = HookState.STRAIGHT;
        HookMessage = "Straight: Please move the hook down to calibrate";

      }

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
