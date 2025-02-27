package frc.robot.subsystems.Vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SwerveConstants;

public class VisionSubsystem extends SubsystemBase
{
  private double tx; // Horizontal offset from crosshair to target (-29.8 to 29.8 degrees)
  private double ty; // Vertical offset from crosshair to target (-24.85 to 24.85 degrees)
  private double ta; // Target area (0-100% of image)
  private double txnc; // Horizontal offset from camera center in degrees
  private double tync; // Vertical offset from camera center in degrees
  private boolean hasTarget; // 
  private double targetID; // AprilTag ID number
  private double targetYaw;

  public VisionSubsystem()
  {
    //init the limelight here  
    NetworkTable nTable = NetworkTableInstance.getDefault().getTable("limelight");

    tx = LimelightHelpers.getTX( "limelight" );
    ty = LimelightHelpers.getTY( "limelight" );
    ta = LimelightHelpers.getTA( "limelight" );
    hasTarget = LimelightHelpers.getTV( "limelight" );
    txnc = LimelightHelpers.getTXNC( "limelight" );
    tync = LimelightHelpers.getTYNC( "limelight" );
    targetID = LimelightHelpers.getFiducialID( "limelight" );
  }

  private void UpdateLLTables()
  {
    tx = LimelightHelpers.getTX( "limelight" );
    ty = LimelightHelpers.getTY( "limelight" );
    ta = LimelightHelpers.getTA( "limelight" );
    hasTarget = LimelightHelpers.getTV( "limelight" );
    txnc = LimelightHelpers.getTXNC( "limelight" );
    tync = LimelightHelpers.getTYNC( "limelight" );
    targetID = LimelightHelpers.getFiducialID( "limelight" );


  }

  



  /* Taken from LL github */
  // simple proportional turning control with Limelight.
  // "proportional control" is a control algorithm in which the output is proportional to the error.
  // in this case, we are going to return an angular velocity that is proportional to the 
  // "tx" value from the Limelight.
  public double limelight_aim_proportional()
  {    
    // kP (constant of proportionality)
    // this is a hand-tuned number that determines the aggressiveness of our proportional control loop
    // if it is too high, the robot will oscillate.
    // if it is too low, the robot will never reach its target
    // if the robot never turns in the correct direction, kP should be inverted.
    double kP = .005;

    // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of 
    // your limelight 3 feed, tx should return roughly 31 degrees.
    double targetingAngularVelocity = LimelightHelpers.getTX("limelight") * kP;

    // convert to radians per second for our drive method
    targetingAngularVelocity *= SwerveConstants.DriveConstants.kMaxSpeedMetersPerSecond;

    //invert since tx is positive when the target is to the right of the crosshair
    targetingAngularVelocity *= -1.0;

    return targetingAngularVelocity;
  }

  // public double limelight_turn_proportional()
  // {
  //   double kP = 0.01;
  //   double targetYaw = LimelightHelpers.gety
  // }

  
  // simple proportional ranging control with Limelight's "ty" value
  // this works best if your Limelight's mount height and target mount height are different.
  // if your limelight and target are mounted at the same or similar heights, use "ta" (area) for target ranging rather than "ty"
  public double limelight_range_proportional()
  {    
    double kP = .005;
    double targetingForwardSpeed = LimelightHelpers.getTY("limelight") * kP;
    targetingForwardSpeed *= SwerveConstants.DriveConstants.kMaxSpeedMetersPerSecond;
    targetingForwardSpeed *= -0.6;
    return targetingForwardSpeed;
  }

  @Override
  public void periodic() 
  {
    UpdateLLTables();

    SmartDashboard.putNumber( "Limelight X: ", tx );
    SmartDashboard.putNumber( "Limelight Y: ", ty );
    SmartDashboard.putNumber( "Limelight A: ", ta );
    SmartDashboard.putNumber( "Limelight TxNC", txnc );
    SmartDashboard.putNumber( "Limelight TyNC", tync );
    SmartDashboard.putBoolean( "LimelightHasTarget", hasTarget );
    SmartDashboard.putNumber( "Target ID", targetID );  

    
  }

  public double getLimelightTA()
  {
    return ta;
  }

  public boolean onTarget()
  {
    return Math.abs( getLimelightTA() ) >= 8.5;
  }



  public double getTargetAngle() 
  {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0.0);
  }

  public double getTargetTY() 
  {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0.0);
  }

  public double getTargetTX() 
  {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0.0);
  }

  public double[] get3DPose() 
  {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("camtran").getDoubleArray(new double[6]);
  }

  
}

