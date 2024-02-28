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
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
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
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ChangeAngle;
import frc.robot.commands.ChangeLightsBasedOffState;
import frc.robot.commands.DisableIntake;
import frc.robot.commands.EnableIntake;
import frc.robot.commands.LimelightTarget;
import frc.robot.commands.RumbleOnTarget;
import frc.robot.commands.RunAnglePID;
import frc.robot.commands.RunAngleSimple;
import frc.robot.commands.RunHooks;
import frc.robot.commands.RunIntake;
import frc.robot.commands.RunShooter;
import frc.robot.commands.RunShooterAuto;
import frc.robot.commands.SetColor;
import frc.robot.commands.UpdateTracking;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Audio;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Hooks;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Limelight;

public class RobotContainer {

  //wbecol;vb edwo;lujdbvw oeWSB 

  // Variables
  private static final double MaxSpeed = 3; // 6 meters per second desired top speed
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
  private final SendableChooser<Command> AutoSelect = new SendableChooser<>();
 
  // PID Controllers
  private PIDController AutoDrivePID = new PIDController(1, 0.001, 0);
  private PIDController AutoTurnPID = new PIDController(2, 0, 0);

  // Commands
 
  // Objects for Tele-op Drive
  public SwerveRequest.FieldCentric driveFieldCentric = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.03).withRotationalDeadband(MaxAngularRate * 0.05) // Small deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
  public SwerveRequest.RobotCentric driveRobotCentric = new SwerveRequest.RobotCentric()
      .withDeadband(MaxSpeed * 0.03).withRotationalDeadband(MaxAngularRate * 0.05) // Small deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I DON'T want field-centric
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(MaxSpeed);

  private void configureBindings() {

    //Button Configurations for Controller 1
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

    //Button Configurations for Controller 1
    JoystickButton driver2A = new JoystickButton(Player2Rum, XboxController.Button.kA.value);
    JoystickButton driver2B = new JoystickButton(Player2Rum, XboxController.Button.kB.value);
    JoystickButton driver2X = new JoystickButton(Player2Rum, XboxController.Button.kX.value);
    JoystickButton driver2Y = new JoystickButton(Player2Rum, XboxController.Button.kY.value);
    JoystickButton driver2LB = new JoystickButton(Player2Rum, XboxController.Button.kLeftBumper.value);
    JoystickButton driver2RB = new JoystickButton(Player2Rum, XboxController.Button.kRightBumper.value);
    JoystickButton driver2LS = new JoystickButton(Player2Rum, XboxController.Button.kLeftStick.value);
    JoystickButton driver2RS = new JoystickButton(Player2Rum, XboxController.Button.kRightStick.value);
    JoystickButton driver2Start = new JoystickButton(Player2Rum, XboxController.Button.kStart.value);
    JoystickButton driver2Back = new JoystickButton(Player2Rum, XboxController.Button.kBack.value);

    //Trigger Setup
    BooleanSupplier driver1LTSupplier = new BooleanSupplier() {

      @Override
      public boolean getAsBoolean() {
        if(Player1.getLeftTriggerAxis() > 0.5){
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
        if(Player1.getRightTriggerAxis() > 0.5){
          return true;
        }
        else{
          return false;
        }
      }
    };
    
    Trigger driver1RT = new Trigger(driver1RTSupplier);

     //Trigger Setup
    BooleanSupplier driver2LTSupplier = new BooleanSupplier() {

      @Override
      public boolean getAsBoolean() {
        if(Player2.getLeftTriggerAxis() > 0.5){
          return true;
        }
        else{
          return false;
        }
      }
    };
    Trigger driver2LT = new Trigger(driver1LTSupplier);

    BooleanSupplier driver2RTSupplier = new BooleanSupplier() {

      @Override
      public boolean getAsBoolean() {
        if(Player2.getRightTriggerAxis() > 0.5){
          return true;
        }
        else{
          return false;
        }
      }
    };
    
    Trigger driver2RT = new Trigger(driver1RTSupplier);

    driver1LT.onTrue(new RunIntake(intake, arm, lights, .6));
 
    arm.setDefaultCommand(new RunAnglePID(arm, hooks, Player1Rum, Player2Rum, limelight, lights));

    limelight.setDefaultCommand(new UpdateTracking(limelight));
 
    drivetrain.setDefaultCommand( 
        drivetrain.applyRequest(() -> driveFieldCentric
        .withVelocityX(Player1.getLeftY() * MaxSpeed * .85)  
        .withVelocityY(Player1.getLeftX()* MaxSpeed * .85) 
        .withRotationalRate((-Player1.getRightX() * MaxAngularRate)) 
    ));

    //Button Controls for Player 1

    //Regular Feed
    Player1.leftTrigger().whileTrue(new RunIntake(intake, arm, lights, .5));
   
   //Normal Shooter
    Player1.rightTrigger().whileTrue(new RunShooter(arm, lights, 1));
    
    // Trap Shooter
    Player1.rightStick().whileTrue(new RunShooter(arm, lights, -.2));

    //Reverse Feed
    Player1.y().whileTrue(new RunIntake(intake, arm, lights, -.5));

    //Reset Field Orientation
    Player1.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), new Rotation2d(180)))));//James changed from Leftbumper 2/24/2024
    
   // Player1.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative(limelight.GetPose())));//James changed from Leftbumper 2/24/2024
    //Test limelight pose
    Player1.back().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative(LimelightHelpers.getBotPose2d_wpiBlue("limelight-sh"))));//James changed from Leftbumper 2/24/2024

    //Locks on to Note
    Player1.leftBumper().whileTrue(new ParallelCommandGroup(DriveToGamePiece(), new RunIntake(intake, arm, lights, .5)));
    
    //Hooks go Up?????
    Player2.leftBumper().whileTrue(new RunHooks(hooks, arm, .75));

    //Hooks go Down?????
    Player2.rightBumper().whileTrue(new RunHooks(hooks, arm, -.75));

    // Shooting into the trap
    Player2.start().whileTrue(new RunShooter(arm, lights, -.07));

    //Registers the Telemetry
    drivetrain.registerTelemetry(logger::telemeterize);

    // Limelight target
    Player1.rightBumper().whileTrue( 
        drivetrain.applyRequest(() -> driveFieldCentric
        .withVelocityX(Player1.getLeftY() * MaxSpeed * .6)  
        .withVelocityY(Player1.getLeftX()* MaxSpeed * .6) 
        .withRotationalRate((-Player1.getRightX() * MaxAngularRate - (limelight.dbl_tx / 12))) 
    ));

  }

  public RobotContainer() {
    
    // Set the button bindings
    configureBindings();

    // Close Notes
    AutoSelect.setDefaultOption("Close Notes", new SequentialCommandGroup( 
    
      // Reset Field Orientation
      ResetPoseOnLimelight().withTimeout(.05),

      // Initial Shot
      new RunShooter(arm, lights, 1).withTimeout(.50),

      // Run intake and drive for the second note
      new EnableIntake(intake, arm, lights, .5).withTimeout(.05),
      DriveTrajectory("CenterShoot"),
      new DisableIntake(intake, arm, lights),
 
      // Take the second shot
      new RunShooterAuto(arm, lights, 1).withTimeout(.50),

      // Run intake and drive for the third note
      new EnableIntake(intake, arm, lights, .5),
      DriveTrajectory("CenterShoot2"),
      new DisableIntake(intake, arm, lights),

      // Take the third shot
      new RunShooterAuto(arm, lights, 1).withTimeout(.50),

      // Run intake and drive for the fourth note
      new EnableIntake(intake, arm, lights, .5),
      DriveTrajectory("CenterShoot3"),
      new DisableIntake(intake, arm, lights),

      // Take the fourth shot
      new RunShooterAuto(arm, lights, 1).withTimeout(.50)

    ));

     // Close Notes Triple
    AutoSelect.addOption("Close Notes Triple", new SequentialCommandGroup( 
    
      // Reset Field Orientation
      ResetAutoPoseOnAlliance("CenterShoot"),

      // Initial Shot
      new RunShooter(arm, lights, 1).withTimeout(.50),

      // Run intake and drive for the second note
      new ParallelCommandGroup(
        
        //Drive and intake
        new RunIntake(intake, arm, lights, .5),
        DriveTrajectory("CenterShoot")

      ).withTimeout(4),
 
      // Take the second shot
      new RunShooterAuto(arm, lights, 1).withTimeout(.50),

      // Run intake and drive for the fourth note
      new ParallelCommandGroup(

        //Drive and intake
        new RunIntake(intake, arm, lights, .5),
        DriveTrajectory("CenterShoot3")

      ).withTimeout(4.3),

      // Take the fourth shot
      new RunShooterAuto(arm, lights, 1).withTimeout(.50)

    ));

    // Steal Left
    AutoSelect.addOption("Steal Left", new SequentialCommandGroup( 
    
      // Reset Field Orientation
      ResetAutoPoseOnAlliance("StealLeft"),

      // Initial Shot
      new RunShooter(arm, lights, 1).withTimeout(.50),

      // Run intake and drive for the second note
      new ParallelCommandGroup(
        //Drive and intake
        new RunIntake(intake, arm, lights, .5),
        DriveTrajectory("StealLeft")

      ).withTimeout(4),
 
      // Take the second shot
      new RunShooterAuto(arm, lights, 1).withTimeout(.50),

      // Run intake and drive for the third note
      new ParallelCommandGroup(
        //Drive and intake
        new RunIntake(intake, arm, lights, .5),
        DriveTrajectory("CloseLeft")

      ).withTimeout(4.3),

      // Take the third shot
      new RunShooterAuto(arm, lights, 1).withTimeout(.50),
 
      DriveTrajectory("LineupLeft")
 
    ));

    // Steal Right
    AutoSelect.addOption("Steal Right", new SequentialCommandGroup( 
    
      // Reset Field Orientation
      ResetAutoPoseOnAlliance("StealRight"),

      // Initial Shot
      new RunShooter(arm, lights, 1).withTimeout(.50),

      // Run intake and drive for the second note
      new ParallelCommandGroup(
        //Drive and intake
        new RunIntake(intake, arm, lights, .5),
        DriveTrajectory("StealRight1")

      ).withTimeout(4),

      new ParallelCommandGroup(
        //Drive and intake
        new RunIntake(intake, arm, lights, .5),
        DriveToGamePiece()

      ).withTimeout(3),
 
        // Run intake and drive for the second note
      new ParallelCommandGroup(
        //Drive and intake
        new RunIntake(intake, arm, lights, .5),
        DriveTrajectory("StealRight2")

      ).withTimeout(4),

      // Take the second shot
      new RunShooterAuto(arm, lights, 1).withTimeout(.50),

      // Run intake and drive for the third note
      new ParallelCommandGroup(
        //Drive and intake
        new RunIntake(intake, arm, lights, .5),
        DriveTrajectory("CloseRight")

      ).withTimeout(4.3),

      // Take the third shot
      new RunShooterAuto(arm, lights, 1).withTimeout(.50),
 
      DriveTrajectory("LineupRight")
 
    ));
 
    // Publish the selectors
    SmartDashboard.putData(AutoSelect);
 
  }
  //Field Orientation Reset
  public Command getAutonomousCommand() {

    // What auto are we running
    //return AutoSelect.getSelected();
    return AutoSelect.getSelected();
    
  }
 
  //Limelight horizontal offset of 13
  public double LimelightRotate() {

    return limelight.HorizonalOffset_LI() / 13;
 
  }

  //Locks on to target when it's spotted
  public Command DriveToGamePiece() {
   
    return drivetrain.applyRequest(() -> driveRobotCentric
      .withVelocityX(1.5)  
      .withVelocityY(0)  
      .withRotationalRate(-limelight.dbl_tx_ri / 16));  
 
  }

  // Trajectory command generator
  public Command DriveTrajectory(String Trajectory) {

    Optional<Alliance> RobotAlliance;
    RobotAlliance = DriverStation.getAlliance();

    return Choreo.choreoSwerveCommand(
      Choreo.getTrajectory("CenterShoot"), 
      () -> (drivetrain.getState().Pose), 
      AutoDrivePID, AutoDrivePID, AutoTurnPID, 
      (ChassisSpeeds speeds) -> drivetrain.setControl(new SwerveRequest.ApplyChassisSpeeds().withSpeeds(speeds)),
      () -> RobotAlliance.get() == Alliance.Red, 
      drivetrain
      );

  }

  // Pose reset function that flips on color
  public Command ResetAutoPoseOnAlliance(String Trajectory) {
 
    Optional<Alliance> RobotAlliance;
    RobotAlliance = DriverStation.getAlliance();

    if (RobotAlliance.get() == Alliance.Red) {
 
      return drivetrain.runOnce(() -> drivetrain.seedFieldRelative(new Pose2d(
      16.565 - Choreo.getTrajectory(Trajectory).getInitialPose().getX(),
      Choreo.getTrajectory(Trajectory).getInitialPose().getY(),
     // Choreo.getTrajectory(Trajectory).getInitialPose().getRotation().minus(new Rotation2d(3.1415)))));
      new Rotation2d(-Math.PI).minus(Choreo.getTrajectory(Trajectory).getInitialPose().getRotation()))));
    
    } else {
 
      return drivetrain.runOnce(() -> drivetrain.seedFieldRelative(Choreo.getTrajectory(Trajectory).getInitialPose()));

    }
 
  }

  public Command ResetPoseOnLimelight() {

    return drivetrain.runOnce(() -> drivetrain.seedFieldRelative(limelight.GetPose()));

  }

}
