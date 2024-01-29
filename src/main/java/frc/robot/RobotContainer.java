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
  private static double[] InitalPose;
 
  // Controllers
  private final CommandXboxController Player1 = new CommandXboxController(0);
 
  // Subsystems
  private final SwerveTeleop drivetrain = TunerConstants.DriveTrain;  
  private final Limelight limelight = new Limelight();

  // Selectors
  private final SendableChooser<Command> AutoSelect = new SendableChooser<>();
  private final SendableChooser<double[]> PoseSelect = new SendableChooser<>();

  // PID Controllers
  private PIDController AutoDrivePID = new PIDController(2, 0, 0);
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
 
    drivetrain.registerTelemetry(logger::telemeterize);

  }

  public RobotContainer() {
    
    // Set the button bindings
    configureBindings();

    // Push telemetry and selectors
    AutoSelect.setDefaultOption("Blue Right Steal", new SequentialCommandGroup(
     
      ResetAutoOdom().withTimeout(.1),
      DriveToPoint(Constants.WayPoints.RightBlueStageLine, .3),
      DriveToPoint(Constants.WayPoints.MiddleRing5FromBlue), 
      DriveToPoint(Constants.WayPoints.RightBlueStageLine, .5), 
      DriveToPoint(Constants.WayPoints.BlueStartingRight)
 
    ));

    AutoSelect.addOption("Blue Left Shoot Three", new SequentialCommandGroup(
     
      ResetAutoOdom().withTimeout(.1),
      DriveToPoint(Constants.WayPoints.BlueThreeShootStart1),
      DriveToPoint(Constants.WayPoints.BlueThreeShootStart2),
      DriveToPoint(Constants.WayPoints.BlueLeftRing),
      DriveToPoint(Constants.WayPoints.BlueCenterRing),
      DriveToPoint(Constants.WayPoints.BlueRightRing)
 
    ));


    SmartDashboard.putData("Select Auto", AutoSelect);

    PoseSelect.setDefaultOption("Default", Constants.WayPoints.FieldCenter);
    PoseSelect.addOption("Blue Left", Constants.WayPoints.BlueStartingLeft);
    PoseSelect.addOption("Blue Center", Constants.WayPoints.BlueStartingCenter);
    PoseSelect.addOption("Blue Right", Constants.WayPoints.BlueStartingRight);
    PoseSelect.addOption("Red Left", Constants.WayPoints.RedStartingLeft);
    PoseSelect.addOption("Red Center", Constants.WayPoints.RedStartingCenter);
    PoseSelect.addOption("Red Right", Constants.WayPoints.RedStartingRight);

    SmartDashboard.putData("Select Starting Position", PoseSelect);
 
    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(
        new Translation2d(Array.getDouble(PoseSelect.getSelected(), 0), Array.getDouble(PoseSelect.getSelected(), 1)), 
        Rotation2d.fromDegrees(Array.getDouble(PoseSelect.getSelected(), 2))));
    }

    InitalPose = PoseSelect.getSelected();
   
  }

  public Command getAutonomousCommand() {

    // [0] = X, [1] = Y, [2] = Rotation
    return AutoSelect.getSelected();
 
  }
 
  public double VerticalMovement(double X) {
 
    double OffsetTarget = X;
    double OffsetCurrent = logger.returnPose().getY();
 
    if (Math.abs(OffsetCurrent - OffsetTarget) > Constants.POSETOLERANCE) {

      //return 0;
      return AutoDrivePID.calculate(OffsetCurrent, OffsetTarget);
    
    } else {

      return 0;

    }
 
  }

  public double VerticalMovement(double X, double Tolerance) {
 
    double OffsetTarget = X;
    double OffsetCurrent = logger.returnPose().getY();
 
    if (Math.abs(OffsetCurrent - OffsetTarget) > Tolerance) {

      //return 0;
      return AutoDrivePID.calculate(OffsetCurrent, OffsetTarget);
    
    } else {

      return 0;

    }
 
  }

  public double HorizonalMovement(double Y) {
 
    double OffsetTarget = Y;
    double OffsetCurrent = logger.returnPose().getX();
 
    if (Math.abs(OffsetCurrent - OffsetTarget) > Constants.POSETOLERANCE) {
    
      return AutoDrivePID.calculate(OffsetCurrent, OffsetTarget);
 
    } else {

      return 0;

    }
 
  }

  public double HorizonalMovement(double Y, double Tolerance) {
 
    double OffsetTarget = Y;
    double OffsetCurrent = logger.returnPose().getX();
 
    if (Math.abs(OffsetCurrent - OffsetTarget) > Tolerance) {
    
      return AutoDrivePID.calculate(OffsetCurrent, OffsetTarget);
 
    } else {

      return 0;

    }
 
  }

  public double Rotate(double TargetAngle) {

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

  public Command DriveToPoint(double[] Pose) {

    double X = Array.getDouble(Pose, 0);
    double Y = Array.getDouble(Pose, 1);
    double Angle = Array.getDouble(Pose, 2);

    return drivetrain.applyRequest(() -> drive
    .withVelocityX(HorizonalMovement(Y))  
    .withVelocityY(VerticalMovement(X)) 
    .withRotationalRate(Rotate(Angle))).until(logger.CheckIfFinished(Y, X, Angle));
  
  }

  public Command DriveToPoint(double[] Pose, double Tolerance) {

    double X = Array.getDouble(Pose, 0);
    double Y = Array.getDouble(Pose, 1);
    double Angle = Array.getDouble(Pose, 2);

    return drivetrain.applyRequest(() -> drive
    .withVelocityX(HorizonalMovement(Y, Tolerance))  
    .withVelocityY(VerticalMovement(X, Tolerance)) 
    .withRotationalRate(Rotate(Angle))).until(logger.CheckIfFinished(Y, X, Angle, Tolerance));
  
  }

  public Command ResetAutoOdom() {

    return drivetrain.runOnce(() -> drivetrain.seedFieldRelative(new Pose2d(
    new Translation2d(Array.getDouble(PoseSelect.getSelected(), 1), Array.getDouble(PoseSelect.getSelected(), 0)), 
    Rotation2d.fromDegrees(Array.getDouble(PoseSelect.getSelected(), 2)))));

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
