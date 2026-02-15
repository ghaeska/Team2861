package frc.robot.subsystems.Vision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.Vision.LimelightHelpers.LimelightTarget_Fiducial;




public class AprilTagLock 
{
  NetworkTable m_networkTable = NetworkTableInstance.getDefault().getTable("limelight");
  private PIDController m_LLPidController;


  public AprilTagLock()
  {
    m_LLPidController = new PIDController(0.01, 0.02, 0.001);
    m_LLPidController.setTolerance(.25);
    m_LLPidController.enableContinuousInput(0, 360);
    m_LLPidController.setSetpoint(0);  
  }

  // public Double getAngleToTag( int tagID )
  // {
  //   LimelightHelpers.LimelightResults results = LimelightHelpers.getLatestResults("limelight").targetingResults;

  //   for( LimelightTarget_Fiducial target: results.targets_Fiducials )
  //   {
  //     if( target.fiducialID == tagID )
  //     {
  //       return PIDController.calculate( target.tx );
  //     }
  //   }
  //   return null;
  // }












}
