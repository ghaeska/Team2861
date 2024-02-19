// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix6.StatusSignal;
//CTRE Imports
import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.SwerveConstants.DriveConstants;
import frc.utils.SwerveUtils;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase 
{
  private double targetHeadingDegrees;
  //private final Pigeon2 m_gyro;
 protected final StatusSignal<Double> m_yawGetter;
  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset );

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  private final RelativeEncoder m_FrontRightEncoder = m_frontRight.getDriveMotorEncoder();
  private final RelativeEncoder m_FrontLeftEncoder = m_frontLeft.getDriveMotorEncoder();
  private final RelativeEncoder m_BackRightEncoder = m_rearRight.getDriveMotorEncoder();
  private final RelativeEncoder m_BackLeftEncoder = m_rearLeft.getDriveMotorEncoder();

  // The gyro sensor
  public static final Pigeon2 m_gyro = new Pigeon2( DriveConstants.kGyroCanId );
  //m_gyro = new Pigeon2( DriveConstants.kGyroCanId );
  //m_yawGetter = m_gyro.getYaw().clone();

  //private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);

  // Slew rate filter variables for controlling lateral acceleration
  private double m_currentRotation = 0.0;
  private double m_currentTranslationDir = 0.0;
  private double m_currentTranslationMag = 0.0;

  private SlewRateLimiter m_magLimiter = new SlewRateLimiter( DriveConstants.kMagnitudeSlewRate );
  private SlewRateLimiter m_rotLimiter = new SlewRateLimiter( DriveConstants.kRotationalSlewRate );
  private double m_prevTime = WPIUtilJNI.now() * 1e-6;

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      Rotation2d.fromDegrees( m_gyro.getAngle() * ( DriveConstants.kGyroReversed ? -1.0:1.0 ) ),
      new SwerveModulePosition[] 
      {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      });

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() 
  {
    
    m_yawGetter = m_gyro.getYaw().clone();
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        Rotation2d.fromDegrees( m_gyro.getAngle() * ( DriveConstants.kGyroReversed ? -1.0:1.0 ) ),
        new SwerveModulePosition[]
        {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });

         /* Velocity Printouts */
     SmartDashboard.putNumber("Front Left Velocity", m_FrontLeftEncoder .getVelocity());
     SmartDashboard.putNumber("Back Left Velocity", m_BackLeftEncoder .getVelocity());
     SmartDashboard.putNumber("Front Right Velocity", m_FrontRightEncoder .getVelocity());
     SmartDashboard.putNumber("Back Right Velocity", m_BackRightEncoder .getVelocity());
    /* Encoder Position Printouts */
    SmartDashboard.putNumber("Front Left Meters", m_FrontLeftEncoder.getPosition());
    SmartDashboard.putNumber("Back Left Meters", m_BackLeftEncoder .getPosition());
    SmartDashboard.putNumber("Front Right Meters", m_FrontRightEncoder .getPosition());
    SmartDashboard.putNumber("Back Right Meters", m_BackRightEncoder .getPosition());
    /* Gyro Printout */
    SmartDashboard.putNumber("Gyro", m_gyro.getAngle());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose()
  {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose)
  {
    m_odometry.resetPosition(
      Rotation2d.fromDegrees( m_gyro.getAngle() * ( DriveConstants.kGyroReversed ? -1.0:1.0 ) ),
      new SwerveModulePosition[]
      {
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_rearLeft.getPosition(),
        m_rearRight.getPosition()
      },
      pose
    );
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   * @param rateLimit     Whether to enable rate limiting for smoother control.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {
    
    double xSpeedCommanded;
    double ySpeedCommanded;

    if( rateLimit )
    {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2( ySpeed, xSpeed );
      double inputTranslationMag = Math.sqrt( Math.pow( xSpeed, 2 ) + Math.pow( ySpeed, 2 ) );

      // Calculate the direction slew rate based on an estimate of the lateral acceleration
      double directionSlewRate;
      if( m_currentTranslationMag != 0.0 )
      {
        directionSlewRate = Math.abs( DriveConstants.kDirectionSlewRate / m_currentTranslationMag );
      } 
      else
      {
        directionSlewRate = 500.0; //some high number that means the slew rate is effectively instantaneous
      }      

      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - m_prevTime;
      double angleDif = SwerveUtils.AngleDifference( inputTranslationDir,
                                                     m_currentTranslationDir );
      if( angleDif < ( 0.45 * Math.PI ) )
      {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular( m_currentTranslationDir, 
                                                                   inputTranslationDir, 
                                                                   ( directionSlewRate * elapsedTime ) );
        m_currentTranslationMag = m_magLimiter.calculate( inputTranslationMag );
      }
      else if( angleDif > ( 0.85 * Math.PI ) ) 
      {
        if( m_currentTranslationMag > 1e-4 ) 
        { //some small number to avoid floating-point errors with equality checking
          // keep currentTranslationDir unchanged
          m_currentTranslationMag = m_magLimiter.calculate(0.0);
        }
        else
        {
          m_currentTranslationDir = SwerveUtils.WrapAngle( m_currentTranslationDir + Math.PI );
          m_currentTranslationMag = m_magLimiter.calculate( inputTranslationMag );
        }
      }
      else {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular( m_currentTranslationDir, 
                                                                   inputTranslationDir, 
                                                                   ( directionSlewRate * elapsedTime )
                                                                 );
        m_currentTranslationMag = m_magLimiter.calculate(0.0);
      }
      m_prevTime = currentTime;
      
      xSpeedCommanded = m_currentTranslationMag * Math.cos( m_currentTranslationDir );
      ySpeedCommanded = m_currentTranslationMag * Math.sin( m_currentTranslationDir );
      m_currentRotation = m_rotLimiter.calculate( rot );


    }
    else
    {
      xSpeedCommanded = xSpeed;
      ySpeedCommanded = ySpeed;
      m_currentRotation = rot;
    }

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = m_currentRotation * DriveConstants.kMaxAngularSpeed;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates
    (
      fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds( xSpeedDelivered, 
                                                              ySpeedDelivered, 
                                                              rotDelivered, 
                                                              Rotation2d.fromDegrees( m_gyro.getAngle() * ( DriveConstants.kGyroReversed ? -1.0:1.0 ) ) )
                    : new ChassisSpeeds( xSpeedDelivered, 
                                         ySpeedDelivered, 
                                         rotDelivered )
    );

    SwerveDriveKinematics.desaturateWheelSpeeds(
      swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond );

    m_frontLeft.setDesiredState( swerveModuleStates[0] );
    m_frontRight.setDesiredState( swerveModuleStates[1] );
    m_rearLeft.setDesiredState( swerveModuleStates[2] );
    m_rearRight.setDesiredState( swerveModuleStates[3] );
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState( new SwerveModuleState( 0, 
                                  Rotation2d.fromDegrees( 45 ) ) );
    m_frontRight.setDesiredState( new SwerveModuleState( 0, 
                                  Rotation2d.fromDegrees( -45 ) ) );
    m_rearLeft.setDesiredState( new SwerveModuleState( 0, 
                                Rotation2d.fromDegrees( -45 ) ) );
    m_rearRight.setDesiredState( new SwerveModuleState( 0, 
                                 Rotation2d.fromDegrees( 45 ) ) );
    
    targetHeadingDegrees = getHeadingDegrees();
  }

  public double getHeadingDegrees() 
  {
    
    return Rotation2d.fromDegrees( m_yawGetter.getValue() ).getDegrees();
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates( SwerveModuleState[] desiredStates ) 
  {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond );
    m_frontLeft.setDesiredState( desiredStates[0] );
    m_frontRight.setDesiredState( desiredStates[1] );
    m_rearLeft.setDesiredState( desiredStates[2] );
    m_rearRight.setDesiredState( desiredStates[3] );
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() 
  {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Resets the Yaw of the robot. */
  public void ResetYaw() 
  {
    SmartDashboard.getBoolean("Heading Reset", false );
    //m_gyro.reset();
    //m_gyro.setYaw(0);
    resetYawToAngle(90.0);
  }

  public void resetYawToAngle(double yawDeg) 
  {
    double curYawDeg = m_yawGetter.getValue();
    double offsetToTargetDeg = targetHeadingDegrees - curYawDeg;
    m_gyro.setYaw(yawDeg);
    Pose2d curPose = getPose();
    Pose2d resetPose = new Pose2d(curPose.getTranslation(), Rotation2d.fromDegrees(yawDeg));
    m_odometry.resetPosition(Rotation2d.fromDegrees(yawDeg), getModulePositions(), resetPose);
    targetHeadingDegrees = yawDeg + offsetToTargetDeg;
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() 
  {
    /* Add the ability to print out the heading into the smartdashboard. */
    return Rotation2d.fromDegrees( m_gyro.getAngle() * ( DriveConstants.kGyroReversed ? -1.0:1.0 ) ).getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() 
  {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public SwerveModulePosition[] getModulePositions() 
  {
    return new SwerveModulePosition[] {
      m_frontLeft.getPosition(),
      m_frontRight.getPosition(),
      m_rearLeft.getPosition(),
      m_rearRight.getPosition()
    };
  }

}
