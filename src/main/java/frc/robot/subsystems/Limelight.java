/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
 
public class Limelight extends SubsystemBase {


  public static String limelightDashboard;
 
  boolean m_LimelightHasValidTarget = false;
  double m_LimelightDriveCommand = 0.0;
  double m_LimelightSteerCommand = 0.0;

  // Local variables to store network values
  boolean b_tv_sh = false;
  boolean b_tv_li = false;
  boolean b_tv_ri = false;
  double dbl_tv_sh;
  double dbl_tv_li;
  double dbl_tv_ri;
  double dbl_ta_sh;
  double dbl_ta_li;
  double dbl_ta_ri;
  double dbl_tx, dbl_tv, dbl_ty, dbl_ta, dbl_ts, dbl_thor, dbl_tvert, dbl_tshort, dbl_tlong;
  public double dbl_tx_li;
  public double dbl_tx_ri;

  //AprilTag specific values
  double[] dbl_botpose;
  double[] dbl_botpose_rear_limelight;
  double[] dbl_campose;
  double dbl_tid;
 
  public boolean HasValidTargetShooter() {
    return b_tv_sh;
  }

  public boolean HasValidTargetRightIntake() {
    return b_tv_ri;
  }

  public boolean HasValidTargetLeftIntake() {
    return b_tv_li;
  }

  public double HorizonalOffset_LI(){

    return dbl_tx_li;

  }

  public double HorizonalOffset_RI(){

    return dbl_tx_ri;

  }

  public void ReadNetworkTables() {

    double dbl_tv_SH = NetworkTableInstance.getDefault().getTable("limelight-sh").getEntry("tv").getDouble(0);
    if (dbl_tv > 0) {
      b_tv_sh = true;
    } else {
      b_tv_sh = false;
    }

    double dbl_tv_RI = NetworkTableInstance.getDefault().getTable("limelight-sh)").getEntry("tv").getDouble(0);
    if (dbl_tv > 0) {
      b_tv_ri = true;
    } else {
      b_tv_ri = false;
    }

    double dbl_tv_LI = NetworkTableInstance.getDefault().getTable("limelight-sh").getEntry("tv").getDouble(0);
    if (dbl_tv > 0) {
      b_tv_li = true;
    } else {
      b_tv_li = false;
    }

    dbl_tx = NetworkTableInstance.getDefault().getTable("limelight-sh").getEntry("tx").getDouble(0);
    dbl_ty = NetworkTableInstance.getDefault().getTable("limelight-sh").getEntry("ty").getDouble(0);
    dbl_ta = NetworkTableInstance.getDefault().getTable("limelight-sh").getEntry("ta").getDouble(0);
    dbl_ts = NetworkTableInstance.getDefault().getTable("limelight-sh").getEntry("ts").getDouble(0);
    dbl_tshort = NetworkTableInstance.getDefault().getTable("limelight-sh").getEntry("tshort").getDouble(0);
    dbl_tlong = NetworkTableInstance.getDefault().getTable("limelight-sh").getEntry("tlong").getDouble(0);
    dbl_thor = NetworkTableInstance.getDefault().getTable("limelight-sh").getEntry("thor").getDouble(0);
    dbl_tvert = NetworkTableInstance.getDefault().getTable("limelight-sh").getEntry("tvert").getDouble(0);

    dbl_tx_li = NetworkTableInstance.getDefault().getTable("limelight-li").getEntry("tx").getDouble(0);
    dbl_tx_ri = NetworkTableInstance.getDefault().getTable("limelight-ri").getEntry("tx").getDouble(0);

    //AprilTag specific calls
    dbl_botpose = NetworkTableInstance.getDefault().getTable("limelight-sh").getEntry("botpose").getDoubleArray(new double[6]);
    dbl_campose = NetworkTableInstance.getDefault().getTable("limelight-sh").getEntry("campose").getDoubleArray(new double[6]);
    dbl_tid = NetworkTableInstance.getDefault().getTable("limelight-sh").getEntry("tid").getDouble(0);
 
  }

  public Limelight() {
  
   
  }

  public void EnableLED()
  {
    NetworkTableInstance.getDefault().getTable("limelight-sh").getEntry("ledMode").setNumber(3); //Set LEDs to current pipeline setting
  }

  public void DisableLED()
  {
    NetworkTableInstance.getDefault().getTable("limelight-sh").getEntry("ledMode").setNumber(1); //Set LEDs to current pipeline setting
  }

  public void updateLimelightTracking() {

    final double STEER_K = 0.05; // how hard to turn toward the target
    final double DRIVE_K = 0.00; // how hard to drive fwd toward the target
    final double DESIRED_TARGET_AREA = 13.0; // Area of the target when the robot reaches the wall
    final double MAX_DRIVE = 0.7; // Simple speed limit so we don't drive too fast

    double tv = NetworkTableInstance.getDefault().getTable("limelight_sh").getEntry("tv").getDouble(0);
    double tx = NetworkTableInstance.getDefault().getTable("limelight_sh").getEntry("tx").getDouble(0);
    double ty = NetworkTableInstance.getDefault().getTable("limelight_sh").getEntry("ty").getDouble(0);
    double ta = NetworkTableInstance.getDefault().getTable("limelight_sh").getEntry("ta").getDouble(0);
    
    if (tv < 1.0) {

      m_LimelightHasValidTarget = false;
      m_LimelightDriveCommand = 0.0;
      m_LimelightSteerCommand = 0.0;
      return;

    }

    m_LimelightHasValidTarget = true;

    // Start with proportional steering

    double dead_band = 0.15;
    double steer_cmd = tx * STEER_K;
    if(steer_cmd != 0)
    {
      if(steer_cmd > 0 && steer_cmd < dead_band)
        steer_cmd =dead_band; //dead band
      else if(steer_cmd < 0 && steer_cmd > -dead_band)
        steer_cmd = -dead_band;
    }
    m_LimelightSteerCommand = steer_cmd;

    // try to drive forward until the target area reaches our desired area
    double drive_cmd = (DESIRED_TARGET_AREA - ta) * DRIVE_K;

    // don't let the robot drive too fast into the goal
    if (drive_cmd > MAX_DRIVE) {
      drive_cmd = MAX_DRIVE;
    }
    m_LimelightDriveCommand = drive_cmd;


    System.out.print("tv: ");
    System.out.print(tv);
    System.out.print(" tx ");
    System.out.print(tx);
    System.out.print(" ty ");
    System.out.print(ty);
    System.out.print(" ta ");
    System.out.print(ta);
    System.out.print(" steer ");
    System.out.print(m_LimelightSteerCommand);
    System.out.print(" drive ");
    System.out.print(m_LimelightDriveCommand);

  }
 
  @Override
  public void periodic() {

    ReadNetworkTables();
    limelightDashboard = "Limelight Horizontal/" + dbl_tx + ";";
    limelightDashboard = limelightDashboard + "Limelight Vertical/" + dbl_ty + ";";
    SmartDashboard.putNumber("Limelight Distance", getDistanceToAprilTag());

  }


  public double getDistanceToTarget(){

    double ty = NetworkTableInstance.getDefault().getTable("limelight_sh").getEntry("ty").getDouble(0);
  
    double targetOffsetAngle_Vertical = ty;

    //how many degrees back is your limelight rotated from perfectly vertical?
    double limelightMountAngleDegrees = -30;//was30

    //distance from the center of the Limelight lens to the floor
    double limelightHeightInches = 48.0;

    //distance from the target to the floor
    double goalHeightInches = 14.625;

    double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
    double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

    //calculate distance
    double distanceFromLimelightToGoalInches = (goalHeightInches - limelightHeightInches)/Math.tan(angleToGoalRadians);
    double distanceFromLimelighttoGoalMeters = distanceFromLimelightToGoalInches / 39.37;

    //return in meters
    if (HasValidTargetShooter() == false) {

      return 0;

    } else {

      return distanceFromLimelighttoGoalMeters;

    }

  }


  public double getDistanceToAprilTag(){

    double ty = NetworkTableInstance.getDefault().getTable("limelight_sh").getEntry("ty").getDouble(0);
  
    double targetOffsetAngle_Vertical = ty;

    //how many degrees back is your limelight rotated from perfectly vertical?
    double limelightMountAngleDegrees = 25;//was30

    //distance from the center of the Limelight lens to the floor
    double limelightHeightInches = 23.75;

    //distance from the low tags to the floor
    double tagHeightInches = 53;
 
    double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
    double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

    //calculate distance
    double distanceFromLimelightToGoalInches = -1;
 
    distanceFromLimelightToGoalInches = (tagHeightInches - limelightHeightInches)/Math.tan(angleToGoalRadians);
 
    double distanceFromLimelighttoGoalMeters = distanceFromLimelightToGoalInches / 39.37;
 
      return distanceFromLimelighttoGoalMeters;
 
  }
 
}
