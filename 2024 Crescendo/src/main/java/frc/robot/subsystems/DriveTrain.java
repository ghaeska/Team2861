package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.kauailabs.navx.frc.AHRS;
//import com.pathplanner.lib.auto.AutoBuilder;
//import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.RobotController;
//import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.SwerveConstants;
//import frc.robot.simulation.SimulatableCANSparkMax;
import frc.robot.SwerveConstants.DriveConstants;
import frc.robot.SwerveConstants.ModuleConstants;
import frc.utils.Conversions;
import frc.utils.SwerveUtils;

public class DriveTrain extends Subsystem
{
  private final RelativeEncoder m_FrontRightEncoder;
  private final RelativeEncoder m_FrontLeftEncoder;
  private final RelativeEncoder m_BackRightEncoder;
  private final RelativeEncoder m_BackLeftEncoder;

  //private final PIDController m_PIDController;
private final SwerveSubsytem 
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

  // The gyro sensor
  public static final Pigeon2 m_gyro = new Pigeon2( DriveConstants.kGyroCanId );

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

  private static DriveTrain m_Instance;
  private static PeriodicIO m_PeriodicIO;

  public static DriveTrain getInstance() 
  {
    if( m_Instance == null )
    {
      m_Instance = new DriveTrain();
    }
    return m_Instance;
  }

  public DriveTrain() 
  {
    m_FrontLeftEncoder = m_frontLeft.getDriveMotorEncoder();
    m_FrontRightEncoder = m_frontRight.getDriveMotorEncoder();
    m_BackLeftEncoder = m_rearLeft.getDriveMotorEncoder();
    m_BackRightEncoder = m_rearRight.getDriveMotorEncoder();

    m_FrontLeftEncoder.setPositionConversionFactor( SwerveConstants.ModuleConstants.kDrivingEncoderPositionFactor );
    m_FrontRightEncoder.setPositionConversionFactor( SwerveConstants.ModuleConstants.kDrivingEncoderPositionFactor );
    m_BackLeftEncoder.setPositionConversionFactor( SwerveConstants.ModuleConstants.kDrivingEncoderPositionFactor );
    m_BackRightEncoder.setPositionConversionFactor( SwerveConstants.ModuleConstants.kDrivingEncoderPositionFactor );
    
    m_FrontLeftEncoder.setPositionConversionFactor( SwerveConstants.ModuleConstants.kDrivingEncoderVelocityFactor );
    m_FrontRightEncoder.setPositionConversionFactor( SwerveConstants.ModuleConstants.kDrivingEncoderVelocityFactor );
    m_BackLeftEncoder.setPositionConversionFactor( SwerveConstants.ModuleConstants.kDrivingEncoderVelocityFactor );
    m_BackRightEncoder.setPositionConversionFactor( SwerveConstants.ModuleConstants.kDrivingEncoderVelocityFactor );
    
    m_FrontLeftEncoder.setPosition(0.0);
    m_FrontRightEncoder.setPosition(0.0);
    m_BackLeftEncoder.setPosition(0.0); 
    m_BackRightEncoder.setPosition(0.0); 

    m_PeriodicIO = new PeriodicIO();

  }

  private static class PeriodicIO 
  {
    //DifferentialDriveWheelSpeeds diffWheelSpeeds = new DifferentialDriveWheelSpeeds(0.0, 0.0);
    //boolean slowMode = false;
    //boolean speedMode = false;
    //double leftVoltage = 0.0;
    //double rightVoltage = 0.0;
  }

  @Override
  public void periodic() 
  {
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

        //SmartDashboard.putNumber("Back Left Encoder", m_rearLeft.getDriveMotorEncoder() );
        SmartDashboard.putNumber("Swerve Gyro Angle", m_gyro.getAngle() );
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
  public void setX() 
  {
    m_frontLeft.setDesiredState( new SwerveModuleState( 0, 
                                  Rotation2d.fromDegrees( 45 ) ) );
    m_frontRight.setDesiredState( new SwerveModuleState( 0, 
                                  Rotation2d.fromDegrees( -45 ) ) );
    m_rearLeft.setDesiredState( new SwerveModuleState( 0, 
                                Rotation2d.fromDegrees( -45 ) ) );
    m_rearRight.setDesiredState( new SwerveModuleState( 0, 
                                 Rotation2d.fromDegrees( 45 ) ) );
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

  /** Zeroes the heading of the robot. */
  public void zeroHeading() 
  {
    SmartDashboard.getBoolean("Heading Reset", false );
    //m_gyro.reset();
    m_gyro.setYaw(0);
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() 
  {
    /* Add the ability to print out the heading into the smartdashboard. */
    return Rotation2d.fromDegrees(m_gyro.getAngle() * ( DriveConstants.kGyroReversed ? -1.0:1.0 ) ).getDegrees();
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

  @Override
  public void reset() 
  {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'reset'");
  }

  @Override
  public void writePeriodicOutputs() {
    //mLeftGroup.setVoltage(mPeriodicIO.leftVoltage);
    //mRightGroup.setVoltage(mPeriodicIO.rightVoltage);
  }

  @Override
  public void stop() {
    //mPeriodicIO.diffWheelSpeeds = new DifferentialDriveWheelSpeeds(0.0, 0.0);
  }

  @Override
  public void outputTelemetry() 
  {
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

}
