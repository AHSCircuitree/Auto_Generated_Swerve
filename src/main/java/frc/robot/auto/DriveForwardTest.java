// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SwerveTeleop;
import frc.robot.Telemetry;

public class DriveForwardTest extends Command {

  /** Creates a new DriveToPoint. */ 

  private final SwerveRequest.FieldCentric drive;

  private SwerveTeleop drivetrain;
  private Telemetry tele;

  private double goalX;
  private double speed;

  private double invertedX = 1;

  private String autoMessage;

  public DriveForwardTest(double X, double Speed, SwerveTeleop Drivetrain, Telemetry Tele) {

    drivetrain = Drivetrain;
    tele = Tele;
 
    goalX = X + tele.returnPose().getX();

    speed = Speed;

    drive = new SwerveRequest.FieldCentric()
    .withDeadband(0.2)
    .withRotationalDeadband(0.2)
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
 
    addRequirements(Drivetrain);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
 
    if (goalX - tele.returnPose().getX() < 0) {

      invertedX = -1;

    }

    autoMessage = "Initalized";
    SmartDashboard.putString("Auto Message", autoMessage);
 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    drivetrain.applyRequest(() -> drive
    .withVelocityX(speed * invertedX)  
    .withVelocityY(0) 
    .withRotationalRate(0));

    SmartDashboard.putNumber("Target X", goalX);
    SmartDashboard.putNumber("Current X", tele.returnPose().getX());

    autoMessage = "In Loop";
    SmartDashboard.putString("Auto Message", autoMessage);
 
    drivetrain.registerTelemetry(tele::telemeterize);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    drivetrain.applyRequest(() -> drive
    .withVelocityX(0)  
    .withVelocityY(0) 
    .withRotationalRate(0));

    autoMessage = "Ended";
    SmartDashboard.putString("Auto Message", autoMessage);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
 
    if ((Math.abs(goalX - tele.returnPose().getX()) < .12)) {

      return true;

    } else {

      return false;

    }
   
  }
}
