// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.utils.SwerveUtils;

/* Pathplanner Imports */
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
//import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
//import com.pathplanner.lib.util.PIDConstants;
//import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.util.Units;

import frc.robot.Constants;
import frc.robot.SwerveConstants;
import frc.robot.SwerveConstants.DriveConstants;
import frc.robot.SwerveConstants.ModuleConstants;
import frc.robot.subsystems.Vision.VisionSubsystem;

public class DriveSubsystem extends SubsystemBase 
{
  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

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

  /* These values are only needed if printing to the dashboard. */
  // private final RelativeEncoder m_FrontRightEncoder = m_frontRight.getDriveMotorEncoder();
  // private final RelativeEncoder m_FrontLeftEncoder = m_frontLeft.getDriveMotorEncoder();
  // private final RelativeEncoder m_BackRightEncoder = m_rearRight.getDriveMotorEncoder();
  // private final RelativeEncoder m_BackLeftEncoder = m_rearLeft.getDriveMotorEncoder();

  // The gyro sensor
  public static final Pigeon2 m_gyro = new Pigeon2( DriveConstants.kGyroCanId );

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      Rotation2d.fromDegrees( m_gyro.getYaw().getValueAsDouble() ),
      new SwerveModulePosition[] 
      {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      });
// Slew rate filter variables for controlling lateral acceleration
  private double m_currentRotation = 0.0;
  private double m_currentTranslationDir = 0.0;
  private double m_currentTranslationMag = 0.0;

  private SlewRateLimiter m_magLimiter = new SlewRateLimiter( DriveConstants.kMagnitudeSlewRate );
  private SlewRateLimiter m_rotLimiter = new SlewRateLimiter( DriveConstants.kRotationalSlewRate );
  private double m_prevTime = WPIUtilJNI.now() * 1e-6;

  RobotConfig m_robotconfig;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() 
  {    
    try
    { 
      m_robotconfig = RobotConfig.fromGUISettings();
    }
    catch( Exception e )
    {
      e.printStackTrace();
    }
    configurePathPlanner();
    // Usage reporting for MAXSwerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);

    /* Gyro Stuff */
  }


  @Override
  public void periodic() 
  {
    // Update the odometry in the periodic block
    m_odometry.update
    (
      Rotation2d.fromDegrees( m_gyro.getYaw().getValueAsDouble() ),
      new SwerveModulePosition[] 
      {
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_rearLeft.getPosition(),
        m_rearRight.getPosition()
      }
    );

    SmartDashboard.putNumber( "Robot YAW", m_gyro.getYaw().getValueAsDouble() );
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
    m_odometry.resetPosition
    (
      Rotation2d.fromDegrees
      (
        m_gyro.getYaw().getValueAsDouble()
      ),
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
                                                             Rotation2d.fromDegrees( m_gyro.getYaw().getValueAsDouble() ) )
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
    m_frontLeft.setDesiredState
    (
      new SwerveModuleState
      (
        0, 
        Rotation2d.fromDegrees( 45 ) 
      )
    );
    m_frontRight.setDesiredState
    (
      new SwerveModuleState
      (
        0, 
        Rotation2d.fromDegrees( -45 )
      )
    );
    m_rearLeft.setDesiredState
    (
      new SwerveModuleState
      (
        0, 
        Rotation2d.fromDegrees( -45 )
      )
    );
    m_rearRight.setDesiredState
    (
      new SwerveModuleState
      (
        0,
        Rotation2d.fromDegrees( 45 )
      )
    );
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates( SwerveModuleState[] desiredStates ) 
  {
    SwerveDriveKinematics.desaturateWheelSpeeds
    (
      desiredStates, DriveConstants.kMaxSpeedMetersPerSecond
    );
    m_frontLeft.setDesiredState( desiredStates[ 0 ] );
    m_frontRight.setDesiredState( desiredStates[ 1 ] );
    m_rearLeft.setDesiredState( desiredStates[ 2 ] );
    m_rearRight.setDesiredState( desiredStates[ 3 ] );
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() 
  {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() 
  {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() 
  {
    return Rotation2d.fromDegrees
    ( 
      m_gyro.getYaw().getValueAsDouble() 
    ).getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() 
  {
    return (m_gyro.getAngularVelocityZWorld().getValueAsDouble() );
  }

  public ChassisSpeeds getChassisSpeeds() 
  {
    return DriveConstants.kDriveKinematics.toChassisSpeeds
    (
      // supplier for chassisSpeed, order of motors need to be the same as the consumer of ChassisSpeed
      m_frontLeft.getState(), 
      m_rearLeft.getState(),
      m_frontRight.getState(),
      m_rearRight.getState()
    );
  }

  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) 
  {
    setModuleStates
    (
      DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds)
    );
  }

  private void driveRobotRelative(ChassisSpeeds speeds) 
  {
    drive(speeds, false);
  }

  private void drive(ChassisSpeeds speeds, boolean fieldRelative) 
  {
    if (fieldRelative)
    {
      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getPose().getRotation());
    }
        
    //speeds = ChassisSpeeds.discretize(speeds, LoggedRobot.defaultPeriodSecs);
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    setModuleStates(swerveModuleStates);
  }

  private ChassisSpeeds getRobotRelativeSpeeds() 
  {
    return DriveConstants.kDriveKinematics.toChassisSpeeds( getModuleStates() );
  }

  private SwerveModuleState[] getModuleStates() 
  {
    return new SwerveModuleState[] 
    {
      m_frontLeft.getState(),
      m_frontRight.getState(),
      m_rearLeft.getState(),
      m_rearRight.getState()
    };
  }

  private SwerveModulePosition[] getSwervePositions()
  {
    SwerveModulePosition[] positions = 
    {
      m_frontLeft.getPosition(), 
      m_frontRight.getPosition(),
      m_rearLeft.getPosition(),
      m_rearRight.getPosition()
    };
    return positions;
  }

  private void configurePathPlanner()
  {
    

    AutoBuilder.configure(
      this::getPose,
      this::resetOdometry, 
      this::getRobotRelativeSpeeds, 
      (speeds, feedforwards) -> driveRobotRelative(speeds), 
      new PPHolonomicDriveController( new PIDConstants
                                      ( 
                                        0.04,
                                        0,
                                        0
                                      ), 
                                      new PIDConstants
                                      (
                                        1,
                                        0,
                                        0
                                      )

                                    ), 
      m_robotconfig, 
      () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
      this
    );
  }

  











}