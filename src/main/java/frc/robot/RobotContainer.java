// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.reflect.Array;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
  private final Limelight limelight = new Limelight();

  // Selectors
  private final SendableChooser<Command> AutoSelect = new SendableChooser<>();

  // PID Controllers
  private PIDController AutoDrivePID = new PIDController(1.2, 0, 0);
  private PIDController AutoTurnPID = new PIDController(0.06, 0, 0);

  // Commands
 
  // Objects for Tele-op Drive
  public SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.03).withRotationalDeadband(MaxAngularRate * 0.05) // Small deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(MaxSpeed);

  private void configureBindings() {

    limelight.setDefaultCommand(null);

    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(Deadband(-Player1.getLeftY()) * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(Deadband(-Player1.getLeftX()) * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate((Deadband(-Player1.getRightX()) * MaxAngularRate)) // Drive counterclockwise with negative X (left)
        ));

    Player1.a().whileTrue(drivetrain.applyRequest(() -> brake));
    Player1.b().whileTrue(drivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-Player1.getLeftY(), -Player1.getLeftX()))));
 
    // reset the field-centric heading on left bumper press
    Player1.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(0)))));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(0)));
    }

    drivetrain.registerTelemetry(logger::telemeterize);

  }

  public RobotContainer() {
    
    // Set the button bindings
    configureBindings();

    // Push telemetry and selectors
 
    SmartDashboard.putData("Select Auto", AutoSelect);
    SmartDashboard.putNumber("PID Test", AutoTurnPID.calculate(-45, 90));
 
  }

  public Command getAutonomousCommand() {

    // [0] = X, [1] = Y, [2] = Rotation
    return new SequentialCommandGroup(
    drivetrain.runOnce(() -> drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(0)))).withTimeout(.1),
    DriveToPoint(0, 1.5, 0),
    DriveToPoint(1, 0, 0),
    DriveToPoint(0, 0, 0)
    //DriveToPoint(0, 0, 0),
    //DriveToPointLimelight(0, 0) 

    );

  }
 
  public double VerticalMovement(double X, double speed) {
 
    double OffsetTarget = -X;
    double OffsetCurrent = logger.returnPose().getY();
 
    if (Math.abs(OffsetCurrent - OffsetTarget) > Constants.POSETOLERANCE) {

      //return 0;
      return AutoDrivePID.calculate(OffsetCurrent, OffsetTarget);
    
    } else {

      return 0;

    }
 
  }

  public double HorizonalMovement(double Y, double speed) {
 
    double OffsetTarget = Y;
    double OffsetCurrent = logger.returnPose().getX();
 
    if (Math.abs(OffsetCurrent - OffsetTarget) > Constants.POSETOLERANCE) {
    
      return AutoDrivePID.calculate(OffsetCurrent, OffsetTarget);
 
    } else {

      return 0;

    }
 
  }

  public double Rotate(double TargetAngle, boolean TurningRight) {

    double CurrentAngle = logger.returnPose().getRotation().getDegrees();

    if (Math.abs(TargetAngle - CurrentAngle) > Constants.ANGLETOLERANCE) {

      return AutoTurnPID.calculate(CurrentAngle, TargetAngle);

    } else {

      return 0;

    }
 
  }

  public double LimelightRotate() {

    return limelight.dbl_tx / 13;
 
  }

  public Command DriveToPoint(double X, double Y, double Angle) {

    return drivetrain.applyRequest(() -> drive
    .withVelocityX(HorizonalMovement(Y, 0))  
    .withVelocityY(VerticalMovement(X, 0)) 
    .withRotationalRate(Rotate(-Angle, true))).until(logger.CheckIfFinished(Y, X, -Angle));
  
  }

  public Command DriveToPointLimelight(double X, double Y) {

    return drivetrain.applyRequest(() -> drive
    .withVelocityX(HorizonalMovement(Y, 0))  
    .withVelocityY(VerticalMovement(X, 0)) 
    .withRotationalRate(LimelightRotate()));
  
  }

  public double Deadband(double value) {

    if (value < .1 && value > 0) {

      return 0;

    } else if (value > -.1 && value < 0) {

      return 0;

    } else {

      return value;

    }

  }

}
