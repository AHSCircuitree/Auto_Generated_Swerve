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
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.RumbleOnTarget;
import frc.robot.commands.RunIntake;
import frc.robot.commands.SetColor;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Limelight;

public class RobotContainer {

  // Variables
  private static final double MaxSpeed = 2.5; // 6 meters per second desired top speed
  private static final double MaxAngularRate = Math.PI; // Half a rotation per second max angular velocity
  private static double[] InitalPose;
 
  // Controllers
  private final CommandXboxController Player1 = new CommandXboxController(0);
  private final XboxController Player1Rum = new XboxController(0);
 
  // Subsystems
  private final Drivetrain drivetrain = TunerConstants.DriveTrain;  
  private final Limelight limelight = new Limelight();
  private final Lights lights = new Lights();
  private final Intake intake = new Intake();

  // Selectors
  private final SendableChooser<Command> AutoSelect = new SendableChooser<>();
  private final SendableChooser<double[]> PoseSelect = new SendableChooser<>();

  // PID Controllers
  private PIDController AutoDrivePID = new PIDController(2.3, 0, 0);
  private PIDController AutoTurnPID = new PIDController(0.2, 0, 0);

  // Commands
 
  // Objects for Tele-op Drive
  public SwerveRequest.FieldCentric driveFieldCentric = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.03).withRotationalDeadband(MaxAngularRate * 0.05) // Small deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
  public SwerveRequest.RobotCentric driveRobotCentric = new SwerveRequest.RobotCentric()
      .withDeadband(MaxSpeed * 0.03).withRotationalDeadband(MaxAngularRate * 0.05) // Small deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I DONT want field-centric
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(MaxSpeed);
  private void configureBindings() {

    limelight.setDefaultCommand(new RumbleOnTarget(limelight,lights,  Player1Rum));

    drivetrain.setDefaultCommand( 
        drivetrain.applyRequest(() -> driveFieldCentric
        .withVelocityX(Deadband(-Player1.getLeftY()) * MaxSpeed)  
        .withVelocityY(Deadband(-Player1.getLeftX()) * MaxSpeed) 
        .withRotationalRate((Deadband(-Player1.getRightX()) * MaxAngularRate)) 
    ));

    Player1.b().whileTrue(drivetrain.applyRequest(() -> brake));

    Player1.y().whileTrue(new RunIntake(intake, .5));

    Player1.rightBumper().whileTrue(DriveToGamePiece());
 
    // reset the field-centric heading on left bumper press
    Player1.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(0)))));
 
    drivetrain.registerTelemetry(logger::telemeterize);

  }

  public RobotContainer() {
    
    // Set the button bindings
    configureBindings();

    // Push telemetry and selectors
    AutoSelect.setDefaultOption("Check Position", new SequentialCommandGroup(
     
      ResetAutoOdom().withTimeout(.1)
       
    ));

    AutoSelect.addOption("Blue Right Steal", new SequentialCommandGroup(
     
      ResetAutoOdom().withTimeout(.1),
      DriveToPoint(Constants.WayPoints.RightBlueStageLine, .8),
      DriveToPoint(Constants.WayPoints.MiddleRing5FromBlue, .2), 
      DriveToPoint(Constants.WayPoints.RightBlueStageLine, .6), 
      DriveToPoint(Constants.WayPoints.BlueStealShootingLine, .1),
      DriveToPoint(Constants.WayPoints.RightBlueStageLine, .8),
      DriveToPoint(Constants.WayPoints.MiddleRing4FromBlue, .2),
      DriveToPoint(Constants.WayPoints.RightBlueStageLine, .6),
      DriveToPoint(Constants.WayPoints.BlueStealShootingLine)
      
    ));
    AutoSelect.addOption("Blue Left Shoot Three", new SequentialCommandGroup(
     
      ResetAutoOdom().withTimeout(.1),
      DriveToPoint(Constants.WayPoints.BlueThreeShootStart1, .8),
      DriveToPoint(Constants.WayPoints.BlueThreeShootStart2, .8),
      DriveToPoint(Constants.WayPoints.BlueLeftRing),
      DriveToPoint(Constants.WayPoints.BlueLeftRingShoot),
      new WaitCommand(1), // Simulates Shooting
      DriveToPoint(Constants.WayPoints.BlueCenterRing),
      new WaitCommand(1), // Simulates Shooting
      DriveToPoint(Constants.WayPoints.BlueAboveRightRing, .3),
      DriveToPoint(Constants.WayPoints.BlueRightRing),
      DriveToPoint(Constants.WayPoints.BlueRightRingShoot1, .3),
      DriveToPoint(Constants.WayPoints.BlueRightRingShoot2)
       
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

  public double RotatePID(double TargetAngle) {

    double CurrentAngle = logger.returnPose().getRotation().getDegrees();

    if (Math.abs(TargetAngle - CurrentAngle) > Constants.ANGLETOLERANCE) {

      return AutoTurnPID.calculate(CurrentAngle, TargetAngle);

    } else {

      return 0;

    }
 
  }

   public double Rotate(double Target) {

    double CurrentAngle;  
    double TargetAngle;

    if (logger.returnPose().getRotation().getDegrees() < 0) {

      CurrentAngle = 360 + logger.returnPose().getRotation().getDegrees();

    } else {

      CurrentAngle = logger.returnPose().getRotation().getDegrees();

    }

    if (Target < 0) {

      TargetAngle = 360 + Target;

    } else {

      TargetAngle = Target;

    }

    SmartDashboard.putNumber("Going to Angle:", TargetAngle);
    SmartDashboard.putNumber("Currently at Angle:", CurrentAngle);

    if (Math.abs(TargetAngle - CurrentAngle) > Constants.ANGLETOLERANCE) {

      if (Math.abs(TargetAngle - CurrentAngle) > Math.abs(TargetAngle - (CurrentAngle + 360))) {

        return -LimitSpeed(AutoTurnPID.calculate(CurrentAngle, TargetAngle), -2, 2);

      } else if (Math.abs(TargetAngle - CurrentAngle) > Math.abs(TargetAngle - (CurrentAngle - 360))) {

        return -LimitSpeed(AutoTurnPID.calculate(CurrentAngle, TargetAngle), -2, 2);

      } else {

        return LimitSpeed(AutoTurnPID.calculate(CurrentAngle, TargetAngle), -2, 2);

      }
      
      
    } else {

      return 0;

    }
 
  }

  public double LimelightRotate() {

    return limelight.HorizonalOffset_LI() / 13;
 
  }

  public Command DriveToPoint(double[] Pose) {

    double X = Array.getDouble(Pose, 0);
    double Y = Array.getDouble(Pose, 1);
    double Angle = Array.getDouble(Pose, 2);

    return drivetrain.applyRequest(() -> driveFieldCentric
    .withVelocityX(HorizonalMovement(Y))  
    .withVelocityY(VerticalMovement(X)) 
    .withRotationalRate(Rotate(Angle))).until(logger.CheckIfFinished(Y, X, Angle));
  
  }

  public Command DriveToPoint(double[] Pose, double Tolerance) {

    double X = Array.getDouble(Pose, 0);
    double Y = Array.getDouble(Pose, 1);
    double Angle = Array.getDouble(Pose, 2);

    return drivetrain.applyRequest(() -> driveFieldCentric
    .withVelocityX(HorizonalMovement(Y, Tolerance))  
    .withVelocityY(VerticalMovement(X, Tolerance)) 
    .withRotationalRate(Rotate(Angle))).until(logger.CheckIfFinished(Y, X, Angle, Tolerance));
  
  }

  public Command ResetAutoOdom() {

    return drivetrain.runOnce(() -> drivetrain.seedFieldRelative(new Pose2d(
    new Translation2d(Array.getDouble(PoseSelect.getSelected(), 1), Array.getDouble(PoseSelect.getSelected(), 0)), 
    Rotation2d.fromDegrees(Array.getDouble(PoseSelect.getSelected(), 2)))));

  }

  public Command DriveToGamePiece() {
   
    return drivetrain.applyRequest(() -> driveRobotCentric
      .withVelocityX(1)  
      .withVelocityY(0)  
      .withRotationalRate(-limelight.HorizonalOffset_RI() / 12));  
    
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

  public double LimitSpeed(double speed, double min, double max) {

    if (speed > max) {

      return max;

    } else if (speed < min) {

      return min;

    } else {

      return speed;
      
    }

  }

}
