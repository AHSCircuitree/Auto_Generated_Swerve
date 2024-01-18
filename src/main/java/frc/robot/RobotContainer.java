// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;

public class RobotContainer {

  // Variables
  private static final double MaxSpeed = 2; // 6 meters per second desired top speed
  private static final double MaxAngularRate = Math.PI; // Half a rotation per second max angular velocity

  // Controllers
  private final CommandXboxController Player1 = new CommandXboxController(0);
 
  // Subsystems
  private final SwerveTeleop drivetrain = TunerConstants.DriveTrain;  

  // Selectors
  SendableChooser<Command> AutoSelect = new SendableChooser<>();

  // Commands
 
  // Objects for Tele-op Drive
  public SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.2) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(MaxSpeed);

  private void configureBindings() {

    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-Player1.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-Player1.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-Player1.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

    Player1.a().whileTrue(drivetrain.applyRequest(() -> brake));
    Player1.b().whileTrue(drivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-Player1.getLeftY(), -Player1.getLeftX()))));
 
    // reset the field-centric heading on left bumper press
    Player1.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);

    SmartDashboard.putNumber("Going to X", 0);
    SmartDashboard.putNumber("Currently at X", logger.returnPose().getX());
    SmartDashboard.putNumber("Going to Y", 0);
    SmartDashboard.putNumber("Currently at Y", logger.returnPose().getY());

  }

  public RobotContainer() {
    configureBindings();

    SmartDashboard.putData("Select Auto", AutoSelect);
    //AutoSelect.setDefaultOption("Test1", new DriveForwardTest(2, .6, drivetrain, logger));
    //AutoSelect.addOption("Test2", new DriveForwardTest(2, .3, drivetrain, logger));

  }

  public Command getAutonomousCommand() {

    return new SequentialCommandGroup(
    drivetrain.runOnce(() -> drivetrain.seedFieldRelative()).withTimeout(.1),
    DriveToPoint(0, 0, .7),
    DriveToPoint(0, 1, .7),
    DriveToPoint(0, 0, .7),
    DriveToPoint(0, 2, .7),
    DriveToPoint(0, 0, .7)
    );

  }
 
  public double MoveToX(double X, double speed) {
 
    double OffsetTarget = X;
    double OffsetCurrent = logger.returnPose().getX();

    SmartDashboard.putNumber("Going to X", OffsetTarget);
    SmartDashboard.putNumber("Currently at X", OffsetCurrent);

    if (OffsetCurrent < OffsetTarget && Math.abs(OffsetCurrent - OffsetTarget) > Constants.POSETOLERANCE) {

      return -speed;

    } else if (OffsetCurrent > OffsetTarget && Math.abs(OffsetCurrent - OffsetTarget) > Constants.POSETOLERANCE) {

      return speed;

    } else {

      return 0;

    }
 
  }

  public double MoveToY(double Y, double speed) {
 
    double OffsetTarget = Y;
    double OffsetCurrent = logger.returnPose().getY();

    SmartDashboard.putNumber("Going to Y", OffsetTarget);
    SmartDashboard.putNumber("Currently at Y", OffsetCurrent);

    if (OffsetCurrent < OffsetTarget && Math.abs(OffsetCurrent - OffsetTarget) > Constants.POSETOLERANCE) {

      return speed;

    } else if (OffsetCurrent > OffsetTarget && Math.abs(OffsetCurrent - OffsetTarget) > Constants.POSETOLERANCE) {

      return -speed;

    } else {

      return 0;

    }
 
  }
 
  public Command DriveToPoint(double X, double Y, double Speed) {

    return drivetrain.applyRequest(() -> drive
    .withVelocityX(MoveToY(Y, .7))  
    .withVelocityY(MoveToX(X, .7)) 
    .withRotationalRate(0)).until(logger.CheckIfFinished(X, Y));

  }

}
