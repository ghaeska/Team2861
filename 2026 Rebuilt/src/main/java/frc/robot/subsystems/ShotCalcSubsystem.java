package frc.robot.subsystems;

import java.util.function.Function;

import frc.robot.utils.BallPhysics.ShotSolution;
import frc.robot.utils.ChassisAccelerations;
import frc.robot.utils.ShootOnTheFlyCalculator;
import frc.robot.utils.ShootOnTheFlyCalculator.InterceptSolution;

import static frc.robot.Constants.ShooterConstants.BALL_TRANSFORM_CENTER;
import static frc.robot.Constants.ShooterConstants.DISTANCE_TO_SHOT_SPEED;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.FieldConstants;

public class ShotCalcSubsystem extends SubsystemBase 
{
  private final DriveSubsystem drivesubsystem;

  private Pose3d currentEffectiveTargetPose = Pose3d.kZero;
  private double currentEffectiveYaw;
  private InterceptSolution currentInterceptSolution;

  private Pose3d targetLocation = FieldConstants.Hub.CENTER;
  private double targetDistance = 0.0;
  private double targetSpeedRps = 8;

  // public enum TargetLocations
  // {
  //   hub_center;
  // } 

  public ShotCalcSubsystem( DriveSubsystem drivesubsystem )
  {
    this.drivesubsystem = drivesubsystem;
  }

  public void periodic()
  {
    Pose2d drivetrainPose = drivesubsystem.getPose();

    targetDistance = drivetrainPose.getTranslation().getDistance(targetLocation.toPose2d().getTranslation());
    targetSpeedRps = DISTANCE_TO_SHOT_SPEED.get(targetDistance);

    SmartDashboard.putNumber( "Shot Calc | Target | Distance:", targetDistance );
    //SmartDashboard.putData( "Shot Calc | Target | Location:", hub_center );
    SmartDashboard.putNumber( "Shot Calc | Target | RPS:", targetSpeedRps );

    Pose3d shooterPose = new Pose3d(drivetrainPose).plus(BALL_TRANSFORM_CENTER);

    ChassisSpeeds drivetrainSpeeds = drivesubsystem.getFieldRelativeSpeeds();
    ChassisAccelerations drivetrainAccelerations = drivesubsystem.getFieldRelativeAccelerations();

    currentInterceptSolution = ShootOnTheFlyCalculator.solveShootOnTheFly(shooterPose, targetLocation,
            drivetrainSpeeds, drivetrainAccelerations, targetSpeedRps,
            5, 0.01);

    currentEffectiveTargetPose = currentInterceptSolution.effectiveTargetPose();
    currentEffectiveYaw = currentInterceptSolution.requiredYaw();
    }

    public void setTarget( Pose3d targetLocation, double targetSpeedRps )
    {
      this.targetLocation = targetLocation;
      this.targetSpeedRps = targetSpeedRps;
    }

    public Pose3d getCurrentEffectiveTargetPose()
    {
      return currentEffectiveTargetPose;
    }

    public double getCurrentEffectiveYaw()
    {
      return currentEffectiveYaw;
    }

    public InterceptSolution getInterceptSolution()
    {
      return currentInterceptSolution;
    }






}
