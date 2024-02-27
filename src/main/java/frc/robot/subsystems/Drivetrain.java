package frc.robot.subsystems;

import java.util.function.Supplier;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import com.choreo.lib.ChoreoTrajectoryState;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.generated.TunerConstants;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem
 * so it can be used in command-based projects easily.
 */
public class Drivetrain extends SwerveDrivetrain implements Subsystem {
 
    // Voltage Variables
    public double FrontLeftDriveVoltage;
    public double FrontRightDriveVoltage;
    public double BackLeftDriveVoltage;
    public double BackRightDriveVoltage;
    public double FrontLeftTurnVoltage;
    public double FrontRightTurnVoltage;
    public double BackLeftTurnVoltage;
    public double BackRightTurnVoltage;

    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
     
    public Drivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }
    public Drivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    @Override
    public void periodic() {

        FrontLeftDriveVoltage = TunerConstants.DriveTrain.getModule(0).getDriveMotor().getSupplyVoltage().getValue();
        FrontRightDriveVoltage = TunerConstants.DriveTrain.getModule(1).getDriveMotor().getSupplyVoltage().getValue();
        BackLeftDriveVoltage = TunerConstants.DriveTrain.getModule(2).getDriveMotor().getSupplyVoltage().getValue();
        BackRightDriveVoltage = TunerConstants.DriveTrain.getModule(3).getDriveMotor().getSupplyVoltage().getValue();
        FrontLeftTurnVoltage = TunerConstants.DriveTrain.getModule(0).getSteerMotor().getSupplyVoltage().getValue();
        FrontRightTurnVoltage = TunerConstants.DriveTrain.getModule(1).getSteerMotor().getSupplyVoltage().getValue();
        BackLeftTurnVoltage = TunerConstants.DriveTrain.getModule(2).getSteerMotor().getSupplyVoltage().getValue();
        BackRightTurnVoltage = TunerConstants.DriveTrain.getModule(3).getSteerMotor().getSupplyVoltage().getValue();
 
        // FL = 0, FR = 1, RL = 2, RR = 3
        SmartDashboard.putNumber("Front Left Drive Voltage", TunerConstants.DriveTrain.getModule(0).getDriveMotor().getSupplyVoltage().getValue());
        SmartDashboard.putNumber("Front Right Drive Voltage", TunerConstants.DriveTrain.getModule(1).getDriveMotor().getSupplyVoltage().getValue());
        SmartDashboard.putNumber("Rear Left Drive Voltage", TunerConstants.DriveTrain.getModule(2).getDriveMotor().getSupplyVoltage().getValue());
        SmartDashboard.putNumber("Rear Right Drive Voltage", TunerConstants.DriveTrain.getModule(3).getDriveMotor().getSupplyVoltage().getValue());

        SmartDashboard.putNumber("Front Left Turn Voltage", TunerConstants.DriveTrain.getModule(0).getSteerMotor().getSupplyVoltage().getValue());
        SmartDashboard.putNumber("Front Right Turn Voltage", TunerConstants.DriveTrain.getModule(1).getSteerMotor().getSupplyVoltage().getValue());
        SmartDashboard.putNumber("Rear Left Turn Voltage", TunerConstants.DriveTrain.getModule(2).getSteerMotor().getSupplyVoltage().getValue());
        SmartDashboard.putNumber("Rear Right Turn Voltage", TunerConstants.DriveTrain.getModule(3).getSteerMotor().getSupplyVoltage().getValue());


    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public Boolean IsDriveTrainGood(){
        
        if (
            TunerConstants.DriveTrain.getModule(0).getDriveMotor().getSupplyVoltage().getValue() < 9 ||
            TunerConstants.DriveTrain.getModule(1).getDriveMotor().getSupplyVoltage().getValue() < 9 ||
            TunerConstants.DriveTrain.getModule(2).getDriveMotor().getSupplyVoltage().getValue() < 9 ||
            TunerConstants.DriveTrain.getModule(0).getDriveMotor().getSupplyVoltage().getValue() < 9 ||
            TunerConstants.DriveTrain.getModule(1).getSteerMotor().getSupplyVoltage().getValue() < 9 ||
            TunerConstants.DriveTrain.getModule(2).getSteerMotor().getSupplyVoltage().getValue() < 9 ||
            TunerConstants.DriveTrain.getModule(0).getSteerMotor().getSupplyVoltage().getValue() < 9 ||
            TunerConstants.DriveTrain.getModule(3).getSteerMotor().getSupplyVoltage().getValue() < 9 ) {

                return false;
                
            } else {

                return true;
            }
    }

}
