// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.reflect.Array;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoControlFunction;
import com.choreo.lib.ChoreoTrajectory;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.PubSubOptions;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ChangeAngle;
 
import frc.robot.commands.RumbleOnTarget;
import frc.robot.commands.RunAnglePID;
import frc.robot.commands.RunAngleSimple;
import frc.robot.commands.RunHooks;
import frc.robot.commands.RunHooksToAngle;
import frc.robot.commands.RunIntake;
import frc.robot.commands.RunShooter;
import frc.robot.commands.SetColor;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Audio;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Hooks;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Limelight;

public class RobotContainer {

  // Variables
  private static final double MaxSpeed = 2.5; // 6 meters per second desired top speed
  private static final double MaxAngularRate = Math.PI; // Half a rotation per second max angular velocity
 
  // Controllers
  private final CommandXboxController Player1 = new CommandXboxController(0);
  private final CommandXboxController Player2 = new CommandXboxController(1);
  private final XboxController Player1Rum = new XboxController(0);
  private final XboxController Player2Rum = new XboxController(1);
 
  // Subsystems
  public final Drivetrain drivetrain = TunerConstants.DriveTrain;  
  public final Limelight limelight = new Limelight();
  public final Lights lights = new Lights();
  public final Intake intake = new Intake();
  public final Hooks hooks = new Hooks();
  public final Arm arm = new Arm();
  public final Audio audio = new Audio();

  // Selectors
  private final SendableChooser<String> AutoSelect = new SendableChooser<>();

  // PID Controllers
  private PIDController AutoDrivePID = new PIDController(.8, 0, 0);
  private PIDController AutoTurnPID = new PIDController(0, 0, 0);

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

    JoystickButton driver1A = new JoystickButton(Player1Rum, XboxController.Button.kA.value);
    JoystickButton driver1B = new JoystickButton(Player1Rum, XboxController.Button.kB.value);
    JoystickButton driver1X = new JoystickButton(Player1Rum, XboxController.Button.kX.value);
    JoystickButton driver1Y = new JoystickButton(Player1Rum, XboxController.Button.kY.value);
    JoystickButton driver1LB = new JoystickButton(Player1Rum, XboxController.Button.kLeftBumper.value);
    JoystickButton driver1RB = new JoystickButton(Player1Rum, XboxController.Button.kRightBumper.value);
    JoystickButton driver1LS = new JoystickButton(Player1Rum, XboxController.Button.kLeftStick.value);
    JoystickButton driver1RS = new JoystickButton(Player1Rum, XboxController.Button.kRightStick.value);
    JoystickButton driver1Start = new JoystickButton(Player1Rum, XboxController.Button.kStart.value);
    JoystickButton driver1Back = new JoystickButton(Player1Rum, XboxController.Button.kBack.value);

    //Trigger Setup
    BooleanSupplier driver1LTSupplier = new BooleanSupplier() {

      @Override
      public boolean getAsBoolean() {
        if(Player1.getLeftTriggerAxis() > 0.2){
          return true;
        }
        else{
          return false;
        }
      }
    };
    Trigger driver1LT = new Trigger(driver1LTSupplier);

    BooleanSupplier driver1RTSupplier = new BooleanSupplier() {

      @Override
      public boolean getAsBoolean() {
        if(Player1.getRightTriggerAxis() > 0.2){
          return true;
        }
        else{
          return false;
        }
      }
    };
    
    Trigger driver1RT = new Trigger(driver1RTSupplier);

    driver1LT.onTrue(new RunIntake(intake, arm, lights, 1));

    limelight.setDefaultCommand(new RumbleOnTarget(limelight, lights,  Player1Rum));
 
    arm.setDefaultCommand(new RunAnglePID(arm, Player1Rum));
 
    drivetrain.setDefaultCommand( 
        drivetrain.applyRequest(() -> driveFieldCentric
        .withVelocityX(Player1.getLeftY() * MaxSpeed)  
        .withVelocityY(Player1.getLeftX()* MaxSpeed) 
        .withRotationalRate((-Player1.getRightX() * MaxAngularRate)) 
    ));

    //Player1.leftTrigger().whileTrue(new RunIntake(intake,arm, .5));
    //Player1.rightTrigger().whileTrue(new RunShooter(arm, SmartDashboard.getNumber("Custom Speed", .5)));
 
    Player1.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(0)))));
    Player1.leftTrigger().whileTrue(new RunHooks(hooks, -.75));
    Player1.rightTrigger().whileTrue(new RunHooks(hooks, .75));

    drivetrain.registerTelemetry(logger::telemeterize);

  }

  public RobotContainer() {
    
    // Set the button bindings
    configureBindings();

    for (int i = 0; i < Constants.UsableTrajectories.length; i++) {
 
      if (i == 0) {

        AutoSelect.setDefaultOption(Constants.UsableTrajectories[i], Constants.UsableTrajectories[i]);

      } else {

        AutoSelect.addOption(Constants.UsableTrajectories[i], Constants.UsableTrajectories[i]);

      }

    }

    SmartDashboard.putData("Select Auto", AutoSelect);

    drivetrain.seedFieldRelative(Choreo.getTrajectory(AutoSelect.getSelected()).getInitialPose());
    //drivetrain.seedFieldRelative(new Pose2d());
 
  }

  public Command getAutonomousCommand() {

    drivetrain.seedFieldRelative(Choreo.getTrajectory(AutoSelect.getSelected()).getInitialPose());

    return  
    Choreo.choreoSwerveCommand(
    Choreo.getTrajectory(AutoSelect.getSelected()), 
    () -> (drivetrain.getState().Pose), 
    AutoDrivePID, AutoDrivePID, AutoTurnPID, 
    (ChassisSpeeds speeds) -> drivetrain.setControl(new SwerveRequest.ApplyChassisSpeeds().withSpeeds(speeds)),
    () -> false, 
    drivetrain
    );
    
  }
 
  public double LimelightRotate() {

    return limelight.HorizonalOffset_LI() / 13;
 
  }

  public Command DriveToGamePiece() {
   
    if (limelight.HorizonalOffset_LI() != 0) {

      return drivetrain.applyRequest(() -> driveRobotCentric
        .withVelocityX(0)  
        .withVelocityY(-1)  
        .withRotationalRate(-limelight.HorizonalOffset_LI() / 12));  

    } else if (limelight.HorizonalOffset_RI() != 0) {

      return drivetrain.applyRequest(() -> driveRobotCentric
        .withVelocityX(0)  
        .withVelocityY(1)  
        .withRotationalRate(-limelight.HorizonalOffset_RI() / 12));  

    } else {

      return drivetrain.applyRequest(() -> driveRobotCentric
        .withVelocityX(0)  
        .withVelocityY(0)  
        .withRotationalRate(0));  

    }
    
  }

}
