// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SwerveTeleop;
import frc.robot.Telemetry;

public class DriveToPoint extends Command {

  /** Creates a new DriveToPoint. */ 

  private final SwerveRequest.FieldCentric drive;

  private SwerveTeleop drivetrain;
  private Telemetry tele;

  private double goalX;
  private double goalY;
  private double angle;
  private double speed;
  private double initialAngle;
  private double invertedX = 1;
  private double invertedY = 1;

  public DriveToPoint(double X, double Y, double Angle, double Speed, SwerveTeleop Drivetrain, Telemetry Tele) {

    
    drivetrain = Drivetrain;
    tele = Tele;

    goalX = X;
    goalY = Y;
    angle = Angle;
    speed = Speed;

    drive = new SwerveRequest.FieldCentric()
    .withDeadband(0.2)
    .withRotationalDeadband(0.2)
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Drivetrain);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
 
    goalX = goalX + tele.returnPose().getX();
    goalY = goalY + tele.returnPose().getY();

    if (goalX < 0) {

      invertedX = -1;

    }

    if (goalY < 0) {

      invertedY = -1;

    }

    initialAngle = drivetrain.getPigeon2().getAngle();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    drivetrain.applyRequest(() -> drive
    .withVelocityX(speed * invertedX)  
    .withVelocityY(speed * invertedY) 
    .withRotationalRate(0));

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    drivetrain.applyRequest(() -> drive
    .withVelocityX(0)  
    .withVelocityY(0) 
    .withRotationalRate(0));

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if ((Math.abs(goalX - tele.returnPose().getX()) > .05) && (Math.abs(goalY - tele.returnPose().getY()) < .05)) {

      return true;

    } else {

      return false;

    }
   
  }
}
