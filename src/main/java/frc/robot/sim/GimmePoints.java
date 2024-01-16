// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SwerveTeleop;

public class GimmePoints extends Command {
  /** Creates a new gimmePoints. */

  private SwerveTeleop drivetrain;

  private Pose2d pointer;
  private double poseX;
  private double poseY;
  private XboxController xbox;

  NetworkTableInstance inst;
  NetworkTable table;
  DoubleArrayPublisher fieldPub;
  StringPublisher fieldTypePub;

  public GimmePoints(XboxController Xbox, SwerveTeleop Drivetrain) {

    xbox = Xbox;
    drivetrain = Drivetrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    inst = NetworkTableInstance.getDefault();
    table = inst.getTable("Fake Pose");
    fieldPub = table.getDoubleArrayTopic("robotPose").publish();
    fieldTypePub = table.getStringTopic(".type").publish();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (xbox.getLeftX() > .1) {

      poseX = poseX + .01;

    } else if (xbox.getLeftX() < -.1) {

      poseX = poseX - .01;

    }

    if (xbox.getLeftY() > .1) {

      poseY = poseY - .01;

    } else if (xbox.getLeftY() < -.1) {

      poseY = poseY + .01;

    }

    pointer = new Pose2d(poseX, poseY, new Rotation2d(0));
   
    fieldTypePub.set("Field2d");
    fieldPub.set(new double[] {
                pointer.getX(),
                pointer.getY(),
                pointer.getRotation().getDegrees()
        });
 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
