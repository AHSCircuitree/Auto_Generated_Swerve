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
import frc.robot.commands.ScoreTrap;
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
    
    //Test limelight pose
    Player1.back().onTrue(ResetPoseOnLimelight());//James changed from Leftbumper 2/24/2024

    //Locks on to Note
    Player1.leftBumper().whileTrue(new ParallelCommandGroup(DriveToGamePiece(), new RunIntake(intake, arm, lights, .5)));
    
    //Drives to shoot
    //Player1.leftStick().whileTrue(DriveToShootTeleop());

    //Hooks go Up
    Player2.leftBumper().whileTrue(new RunHooks(hooks, arm, .75));

    //Hooks go Down
    Player2.rightBumper().whileTrue(new RunHooks(hooks, arm, -.75));

    // Shooting into the trap
    Player2.start().whileTrue(new ScoreTrap(arm));

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
    AutoSelect.setDefaultOption("Blue Center Shoot 4", new SequentialCommandGroup( 
    
      // Reset Field Orientation
      ResetAutoPoseOnAlliance("CenterShoot", false),
      //ResetPoseOnLimelight(),

      // Initial Shot
      new RunShooter(arm, lights, 1).withTimeout(.50),

      // Run intake and drive for the third note
      new ParallelCommandGroup(
        //Drive and intake
        new RunIntake(intake, arm, lights, .5),
        DriveTrajectory("CenterShoot", false)

      ).withTimeout(4.3),
 
      // Take the second shot
      new RunShooter(arm, lights, 1).withTimeout(.50),

      // Run intake and drive for the third note
      new ParallelCommandGroup(
        //Drive and intake
        new RunIntake(intake, arm, lights, .5),
        DriveTrajectory("CenterShoot2", false)

      ).withTimeout(4.3),

      // Take the third shot
      new RunShooterAuto(arm, lights, 1).withTimeout(.50),

      // Run intake and drive for the third note
      new ParallelCommandGroup(
        //Drive and intake
        new RunIntake(intake, arm, lights, .5),
        DriveTrajectory("CenterShoot3", false)

      ).withTimeout(1.7),

      DriveToGamePiece().withTimeout(.43),

       // Run intake and drive for the third note
      new ParallelCommandGroup(
        //Drive and intake
        new RunIntake(intake, arm, lights, .5),
        DriveTrajectory("CenterShoot4", false)

      ).withTimeout(2.3),

      // Take the fourth shot
      new RunShooterAuto(arm, lights, 1).withTimeout(.50)

    ));

    // Close Notes
    AutoSelect.addOption("Red Center Shoot 4", new SequentialCommandGroup( 
    
      // Reset Field Orientation
      ResetAutoPoseOnAlliance("CenterShoot", true),
      //ResetPoseOnLimelight(),

      // Initial Shot
      new RunShooter(arm, lights, 1).withTimeout(.50),

      // Run intake and drive for the third note
      new ParallelCommandGroup(
        //Drive and intake
        new RunIntake(intake, arm, lights, .5),
        DriveTrajectory("CenterShoot", true)

      ).withTimeout(4.3),
 
      // Take the second shot
      new RunShooter(arm, lights, 1).withTimeout(.50),

      // Run intake and drive for the third note
      new ParallelCommandGroup(
        //Drive and intake
        new RunIntake(intake, arm, lights, .5),
        DriveTrajectory("CenterShoot2", true)

      ).withTimeout(4.3),

      // Take the third shot
      new RunShooterAuto(arm, lights, 1).withTimeout(.50),

      // Run intake and drive for the third note
      new ParallelCommandGroup(
        //Drive and intake
        new RunIntake(intake, arm, lights, .5),
        DriveTrajectory("CenterShoot3", true)

      ).withTimeout(1.7),

      DriveToGamePiece().withTimeout(.43),

       // Run intake and drive for the third note
      new ParallelCommandGroup(
        //Drive and intake
        new RunIntake(intake, arm, lights, .5),
        DriveTrajectory("CenterShoot4", true)

      ).withTimeout(2.3),

      // Take the fourth shot
      new RunShooterAuto(arm, lights, 1).withTimeout(.50)

    ));

    // Close Notes
    AutoSelect.addOption("Blue Amp Shoot 2", new SequentialCommandGroup( 
    
      // Reset Field Orientation
      ResetAutoPoseOnAlliance("CloseLeft", false),

      // Initial Shot
      new RunShooter(arm, lights, 1).withTimeout(.50),

      // Run intake and drive for the third note
      new ParallelCommandGroup(
        //Drive and intake
        new RunIntake(intake, arm, lights, .5),
        DriveTrajectory("CloseLeft", false)

      ).withTimeout(4.3),

      new RunShooterAuto(arm, lights, 1).withTimeout(.50)
 
    ));

    
    // Close Notes
    AutoSelect.addOption("Red Amp Shoot 2", new SequentialCommandGroup( 
    
      // Reset Field Orientation
      ResetAutoPoseOnAlliance("CloseLeft", true),

      // Initial Shot
      new RunShooter(arm, lights, 1).withTimeout(.50),

      // Run intake and drive for the third note
      new ParallelCommandGroup(
        //Drive and intake
        new RunIntake(intake, arm, lights, .5),
        DriveTrajectory("CloseLeft", true)

      ).withTimeout(4.3),

      new RunShooter(arm, lights, 1).withTimeout(.50)
 
    ));

     // Close Notes
    AutoSelect.addOption("Blue Amp Shoot 3", new SequentialCommandGroup( 
    
      // Reset Field Orientation
      ResetAutoPoseOnAlliance("CloseLeft", false),

    
      // Run intake and drive for the third note
      new ParallelCommandGroup(
        //Drive and intake
        new RunIntake(intake, arm, lights, .5),
        DriveTrajectory("CloseLeft", false)

      ).withTimeout(4.3),

      // Initial Shot
      new RunShooter(arm, lights, 1).withTimeout(.50),

      // Run intake and drive for the third note
      new ParallelCommandGroup(
        //Drive and intake
        new RunIntake(intake, arm, lights, .5),
        DriveTrajectory("CloseLeftMiddle", false)

      ).withTimeout(4.3),

       // Initial Shot
      new RunShooter(arm, lights, 1).withTimeout(.5)
 
    ));

     // Close Notes
    AutoSelect.addOption("Red Amp Shoot 3", new SequentialCommandGroup( 
    
      // Reset Field Orientation
      ResetAutoPoseOnAlliance("CloseLeft", true),

    
      // Run intake and drive for the third note
      new ParallelCommandGroup(
        //Drive and intake
        new RunIntake(intake, arm, lights, .5),
        DriveTrajectory("CloseLeft", true)

      ).withTimeout(4.3),

      // Initial Shot
      new RunShooterAuto(arm, lights, 1).withTimeout(.50),

      // Run intake and drive for the third note
      new ParallelCommandGroup(
        //Drive and intake
        new RunIntake(intake, arm, lights, .5),
        DriveTrajectory("CloseLeftMiddle", true)

      ).withTimeout(4.3),

       // Initial Shot
      new RunShooterAuto(arm, lights, 1).withTimeout(.5)
 
    ));
  
    // Center Shoot
    AutoSelect.addOption("Blue Center Shoot 1", new SequentialCommandGroup( 
    
      // Reset Field Orientation
      ResetAutoPoseOnAlliance("CenterShoot", false),

      // Initial Shot
      new RunShooter(arm, lights, 1).withTimeout(.50),

           // Run intake and drive for the third note
      new ParallelCommandGroup(
        //Drive and intake
        new RunIntake(intake, arm, lights, .5),
        DriveTrajectory("CenterShoot", false)

      ).withTimeout(4.3),
 
      // Take the second shot
      new RunShooterAuto(arm, lights, 1).withTimeout(.50),

             // Run intake and drive for the third note
      new ParallelCommandGroup(
        //Drive and intake
        new RunIntake(intake, arm, lights, .5),
        DriveTrajectory("LineupMiddle", false)

     )
    
    ));

    // Center Shoot
    AutoSelect.addOption("Red Center Shoot 1", new SequentialCommandGroup( 
    
      // Reset Field Orientation
      ResetAutoPoseOnAlliance("CenterShoot", true),

      // Initial Shot
      new RunShooter(arm, lights, 1).withTimeout(.50),

           // Run intake and drive for the third note
      new ParallelCommandGroup(
        //Drive and intake
        new RunIntake(intake, arm, lights, .5),
        DriveTrajectory("CenterShoot", true)

      ).withTimeout(4.3),
 
      // Take the second shot
      new RunShooterAuto(arm, lights, 1).withTimeout(.50),

             // Run intake and drive for the third note
      new ParallelCommandGroup(
        //Drive and intake
        new RunIntake(intake, arm, lights, .5),
        DriveTrajectory("LineupMiddle", true)

     )
    
    ));


    // Close Right
    AutoSelect.addOption("Blue Close Right", new SequentialCommandGroup( 
    
      // Reset Field Orientation
      ResetAutoPoseOnAlliance("CloseRight", false),

      // Initial Shot
      new RunShooter(arm, lights, 1).withTimeout(.50),

      // Run intake and drive for the third note
      new ParallelCommandGroup(
        //Drive and intake
        new RunIntake(intake, arm, lights, .5),
        DriveTrajectory("CloseRight", false)

      ).withTimeout(4),
 
         // Take the second shot
      new RunShooterAuto(arm, lights, 1).withTimeout(.50),

           // Run intake and drive for the third note
      new ParallelCommandGroup(
        //Drive and intake
        new RunIntake(intake, arm, lights, .5),
        DriveTrajectory("LineupRight", false)

      )

      ));
    
    AutoSelect.addOption("Blue Close Right and Middle", new SequentialCommandGroup(
        
      //Reset Field Orientation 
      ResetAutoPoseOnAlliance("CloseRight", false),

      //Inital shot
      new RunShooter(arm, lights, 1).withTimeout(.50),

           // Run intake and drive for the third note
      new ParallelCommandGroup(
        //Drive and intake
        new RunIntake(intake, arm, lights, .5),
        DriveTrajectory("CloseRight", false)

      ).withTimeout(4),

      //Shoot 2nd note
      new RunShooterAuto(arm, lights, 1).withTimeout(.50),

      // Run intake and drive for the third note
           // Run intake and drive for the third note
      new ParallelCommandGroup(
        //Drive and intake
        new RunIntake(intake, arm, lights, .5),
        DriveTrajectory("CloseRightMiddle", false)

      )
      
      ));

      //Amp Auto
    AutoSelect.addOption("Amp", new SequentialCommandGroup(

      //Reset Field Orinetation
      ResetAutoPoseOnAlliance("Amp", false),
      //Run to amp
      new ParallelCommandGroup(
        DriveTrajectory("Amp", false)     
        ).withTimeout(.4),
      
        //Shoot first note into amp
      new RunShooterAuto(arm, lights, 1).withTimeout(.50),

      //bring arm down here

      //Run intake and drive for second note
      new ParallelCommandGroup(
        //Drive and intake
        new RunIntake(intake, arm, lights, .5),
        DriveTrajectory("Amp", false)
       ),

       //Bring arm up here

       //Shoot second note into amp
      new RunShooterAuto(arm, lights, 1).withTimeout(.50),

      //Bring arm back down here

      //Drive to third not
      new ParallelCommandGroup(
        //Drive and intake
        new RunIntake(intake, arm, lights, .5),
        DriveTrajectory("Amp", false)
      ),

      //Shoot third note onto our side of the field
      new RunShooterAuto(arm, lights, 1).withTimeout(.50),

      //Drive to fourth note
      new ParallelCommandGroup(
        //Drive and intake
        new RunIntake(intake, arm, lights, .5),
        DriveTrajectory("Amp", false)
      )

    ));
      SmartDashboard.putData("Select Auto", AutoSelect);
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
      .withVelocityX(1.7)  
      .withVelocityY(0)  
      .withRotationalRate(-limelight.dbl_tx_ri / 16));  
 
  }

  // Trajectory command generator
  public Command DriveTrajectory(String Trajectory, boolean IsRed) {

    Optional<Alliance> RobotAlliance;
    RobotAlliance = DriverStation.getAlliance();

    return Choreo.choreoSwerveCommand(
      Choreo.getTrajectory(Trajectory), 
      () -> (drivetrain.getState().Pose), 
      AutoDrivePID, AutoDrivePID, AutoTurnPID, 
      (ChassisSpeeds speeds) -> drivetrain.setControl(new SwerveRequest.ApplyChassisSpeeds().withSpeeds(speeds)),
      () -> IsRed, 
      drivetrain
      );

  }

  // Pose reset function that flips on color
  public Command ResetAutoPoseOnAlliance(String Trajectory, boolean IsRed) {
 
    Optional<Alliance> RobotAlliance;
    RobotAlliance = DriverStation.getAlliance();

    if (IsRed == true) {
 
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

    Pose2d LimelightPose =  LimelightHelpers.getBotPose2d_wpiBlue("limelight-sh");
      
    return drivetrain.runOnce(() -> drivetrain.seedFieldRelative(LimelightPose));

  }

  /*
  public SequentialCommandGroup DriveToShootTeleop() {

    Pose2d LimelightPose = new Pose2d(LimelightHelpers.getBotPose2d_wpiBlue("limelight-sh").getX(), 
    LimelightHelpers.getBotPose2d_wpiBlue("limelight-sh").getY(), new Rotation2d(-180));

    return new SequentialCommandGroup(
    //drivetrain.runOnce(() -> drivetrain.seedFieldRelative(LimelightPose)), 
    DriveTrajectory("TeleopShoot"));

  }
  */

}
