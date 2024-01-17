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
  private static final double MaxSpeed = 6; // 6 meters per second desired top speed
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
  }

  public RobotContainer() {
    configureBindings();

    SmartDashboard.putData("Select Auto", AutoSelect);
    //AutoSelect.setDefaultOption("Test1", new DriveForwardTest(2, .6, drivetrain, logger));
    //AutoSelect.addOption("Test2", new DriveForwardTest(2, .3, drivetrain, logger));

  }

  public Command getAutonomousCommand() {

    return new SequentialCommandGroup(
    drivetrain.applyRequest(() -> drive
    .withVelocityX(MoveToX(-3, .7))  
    .withVelocityY(MoveToY(-1, .7)) 
    .withRotationalRate(10)).withTimeout(7),
     drivetrain.applyRequest(() -> drive
    .withVelocityX(MoveToX(0, .7))  
    .withVelocityY(MoveToY(0, .7)) 
    .withRotationalRate(0))
    );

  }

  public double normalizeSpeeds(double speed) {

    if (speed > -.7 && speed < -.1) {

      return -.7;

    } else if (speed < .7 && speed > .1) {

      return .7;

    } else {

      return speed;

    }
 
  }    


  public double MoveToX(double X, double speed) {
 
    double OffsetX = X + Constants.XOFFSET;

    if (logger.returnPose().getX() + Constants.XOFFSET < OffsetX && Math.abs(logger.returnPose().getX() - X) > Constants.POSETOLERANCE) {

      SmartDashboard.putNumber("Going to X", OffsetX);
      SmartDashboard.putNumber("Current At X", logger.returnPose().getX()+ Constants.XOFFSET);
      return speed;

    } else if (logger.returnPose().getX() + Constants.XOFFSET > OffsetX && Math.abs(logger.returnPose().getX() - X) > Constants.POSETOLERANCE) {

      SmartDashboard.putNumber("Going to X", OffsetX);
      SmartDashboard.putNumber("Current At X", logger.returnPose().getX()+ Constants.XOFFSET);
      return -speed;

    } else {

      return 0;

    }

  }

  public double MoveToY(double Y, double speed) {
 
    double OffsetY = Y + Constants.YOFFSET;

    if (logger.returnPose().getY() + Constants.YOFFSET < OffsetY && Math.abs(logger.returnPose().getY() - Y) > Constants.POSETOLERANCE) {

      SmartDashboard.putNumber("Going to Y", OffsetY);
      SmartDashboard.putNumber("Current At Y", logger.returnPose().getY()+ Constants.YOFFSET);
      return speed;

    } else if (logger.returnPose().getY() + Constants.YOFFSET > OffsetY && Math.abs(logger.returnPose().getY() - Y) > Constants.POSETOLERANCE) {

      SmartDashboard.putNumber("Going to Y", OffsetY);
      SmartDashboard.putNumber("Current At Y", logger.returnPose().getY()+ Constants.YOFFSET);
      return -speed;

    } else {

      return 0;

    }

  }

}
