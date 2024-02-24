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
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ChangeAngle;
import frc.robot.commands.ChangeLightsBasedOffState;
import frc.robot.commands.LimelightTarget;
import frc.robot.commands.RumbleOnTarget;
import frc.robot.commands.RunAnglePID;
import frc.robot.commands.RunAngleSimple;
import frc.robot.commands.RunHooks;
import frc.robot.commands.RunHooksToAngle;
import frc.robot.commands.RunIntake;
import frc.robot.commands.RunShooter;
import frc.robot.commands.RunShooterAuto;
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
  private final SendableChooser<String> AutoSelect = new SendableChooser<>();

  // PID Controllers
  private PIDController AutoDrivePID = new PIDController(1, 0, 0);
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

    driver1LT.onTrue(new RunIntake(intake, arm, lights, .6));

    //limelight.setDefaultCommand(new RumbleOnTarget(limelight, lights,  Player1Rum));
    //lights.setDefaultCommand(new ChangeLightsBasedOffState(lights, limelight, Player1Rum));
    arm.setDefaultCommand(new RunAnglePID(arm, hooks, Player1Rum, limelight, lights));
 
    drivetrain.setDefaultCommand( 
        drivetrain.applyRequest(() -> driveFieldCentric
        .withVelocityX(Player1.getLeftY() * MaxSpeed * .8)  
        .withVelocityY(Player1.getLeftX()* MaxSpeed * .8) 
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
    Player1.leftBumper().whileTrue(new RunIntake(intake, arm, lights, -.5));

    //Reset Field Orientation
    Player1.b().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(180)))));//James changed from Leftbumper 2/24/2024
    
    //Locks on to Note
    Player1.rightBumper().whileTrue(new ParallelCommandGroup(DriveToGamePiece(), new RunIntake(intake, arm, lights, .5)));
    
    //Hooks go Up?????
    Player1.start().whileTrue(new RunHooks(hooks, arm, -.75));

    //Hooks go Down?????
    Player1.back().whileTrue(new RunHooks(hooks, arm, .75));

    //Registers the Telemetry
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
  //Field Orientation Reset
  public Command getAutonomousCommand() {

    // Seeds the orientation
    drivetrain.seedFieldRelative(Choreo.getTrajectory("CenterShoot").getInitialPose());

    // The auto command
    SequentialCommandGroup CenterShootFront = new SequentialCommandGroup( 
    
      // Initial Shot
      new RunShooter(arm, lights, 1).withTimeout(.50),
 
      // Run intake and drive
      new ParallelCommandGroup(

      // Intake
      new RunIntake(intake, arm, lights, .5),

      // Drive to second ring
      Choreo.choreoSwerveCommand(
      Choreo.getTrajectory("CenterShoot"), 
      () -> (drivetrain.getState().Pose), 
      AutoDrivePID, AutoDrivePID, AutoTurnPID, 
      (ChassisSpeeds speeds) -> drivetrain.setControl(new SwerveRequest.ApplyChassisSpeeds().withSpeeds(speeds)),
      () -> false, 
      drivetrain
      )

      ).withTimeout(4),
 
      // Take the second shot
      new RunShooterAuto(arm, lights, 1).withTimeout(.50),

      // Run intake and drive for the third note
      new ParallelCommandGroup(

      // Intake
      new RunIntake(intake, arm, lights, .5),

      // Drive
      Choreo.choreoSwerveCommand(
      Choreo.getTrajectory("CenterShoot2"), 
      () -> (drivetrain.getState().Pose), 
      AutoDrivePID, AutoDrivePID, AutoTurnPID, 
      (ChassisSpeeds speeds) -> drivetrain.setControl(new SwerveRequest.ApplyChassisSpeeds().withSpeeds(speeds)),
      () -> false, 
      drivetrain
      )

      ).withTimeout(4.3),

      // Shoot the third note
      new RunShooterAuto(arm, lights, 1).withTimeout(.50),

      // Run intake and drive for the fourth note
      new ParallelCommandGroup(

      // Intake
      new RunIntake(intake, arm, lights, .5),

      // Drive
      Choreo.choreoSwerveCommand(
      Choreo.getTrajectory("CenterShoot3"), 
      () -> (drivetrain.getState().Pose), 
      AutoDrivePID, AutoDrivePID, AutoTurnPID, 
      (ChassisSpeeds speeds) -> drivetrain.setControl(new SwerveRequest.ApplyChassisSpeeds().withSpeeds(speeds)),
      () -> false, 
      drivetrain
      )

      ).withTimeout(4.3),

      // Take the fourth shot
      new RunShooterAuto(arm, lights, 1).withTimeout(.50)

    );

     // The auto command
    SequentialCommandGroup CenterShootBack = new SequentialCommandGroup( 
    
      // Initial Shot
      new RunShooter(arm, lights, 1).withTimeout(1),
 
      // Run intake and drive
      new ParallelCommandGroup(

      // Intake
      new RunIntake(intake, arm, lights, .5),

      // Drive
      Choreo.choreoSwerveCommand(
      Choreo.getTrajectory("CenterShootBack"), 
      () -> (drivetrain.getState().Pose), 
      AutoDrivePID, AutoDrivePID, AutoTurnPID, 
      (ChassisSpeeds speeds) -> drivetrain.setControl(new SwerveRequest.ApplyChassisSpeeds().withSpeeds(speeds)),
      () -> false, 
      drivetrain
      )

      ).withTimeout(4),
 
      // Take the second shot
      new RunShooter(arm, lights, 1).withTimeout(.85)

    );

    // What auto are we running
    return CenterShootFront;
    
  }
 
  //Limelight horizontal offset of 13
  public double LimelightRotate() {

    return limelight.HorizonalOffset_LI() / 13;
 
  }

  //Locks on to target when it's spotted
  public Command DriveToGamePiece() {
   
    if (1 != 0) {

      return drivetrain.applyRequest(() -> driveRobotCentric
        .withVelocityX(.7)  
        .withVelocityY(0)  
        .withRotationalRate(-limelight.dbl_tx_ri / 16));  

    } else {

      return drivetrain.applyRequest(() -> driveRobotCentric
        .withVelocityX(0)  
        .withVelocityY(0)  
        .withRotationalRate(0));  

    }
    
  }

}
