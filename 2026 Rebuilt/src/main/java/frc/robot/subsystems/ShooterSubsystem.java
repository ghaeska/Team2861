package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.SwerveConstants.OIConstants;
import edu.wpi.first.math.MathUtil;

import frc.robot.Constants.ShooterConstants;
import frc.robot.Configs;



public class ShooterSubsystem extends SubsystemBase
{
  private boolean RightMotorRunning = false;
  private boolean LeftMotorRunning = false;

  public boolean CoralPossession = false;

  /* Define the motors */
  private final SparkFlex m_LeftShooterMotor;
  private final SparkFlex m_RightShooterMotor;
  //private final SparkMax m_PivotCoralMotor;

  

  /* Define Relative motor Encoders */
  private RelativeEncoder m_LeftShooterEncoder;
  private RelativeEncoder m_rightShooterEncoder;

  

  public ShooterSubsystem()
  {
    /* Assign the intake motor, they are spark Flex motors */
    m_LeftShooterMotor = new SparkFlex(Constants.ShooterConstants.k_LeftShooterMotorCANId, MotorType.kBrushless );
    m_RightShooterMotor = new SparkFlex(Constants.ShooterConstants.k_RightShooterMotorCANId, MotorType.kBrushless );

    
    /* Setup the motor Encoders. */
    m_LeftShooterEncoder = m_LeftShooterMotor.getEncoder();
    m_rightShooterEncoder = m_RightShooterMotor.getEncoder();

    /* Configure the left motor */
    m_LeftShooterMotor.configure
    (
      Configs.ShooterModule.ShooterSparkFlexConfig, 
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters 
    );
    /* Configure the right motor */
    m_RightShooterMotor.configure
    (
      Configs.ShooterModule.ShooterSparkFlexConfig
      .inverted( true ), 
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters 
    );

   

  }

/************************** Smart Dashboard Values ****************************/
@Override
  public void periodic() 
  {
    /* Print out the Coral Encoder positions and velocities */
    SmartDashboard.putNumber( "LeftCoralSpeed:", m_LeftShooterEncoder.getVelocity() );
    SmartDashboard.putNumber( "RightCoralSpeed:", m_rightShooterEncoder.getVelocity() );    

    SmartDashboard.putNumber( " LeftCoralCurrent", m_LeftShooterMotor.getOutputCurrent() );
    SmartDashboard.putNumber( "RightCoralCurrent", m_RightShooterMotor.getOutputCurrent() );

    SmartDashboard.putBoolean( "IsLeftMotorRunning?", LeftMotorRunning );
    SmartDashboard.putBoolean( "IsRightMotorRunning?", RightMotorRunning );

    SmartDashboard.putBoolean( "Coral Possession", CoralPossession );  
  
  }

/*********************** Helper Functions for Coral ***************************/
  public void runCoralMotor( double voltage )
  {    
    m_LeftShooterMotor.set( voltage );
    m_RightShooterMotor.set( voltage );
    //RightMotorRunning = true;
    //LeftMotorRunning = true;

    // if( CheckRightCoral() || CheckLeftCoral() )
    // {
    //   stopCoral( LED );
    //   //CoralPossession = true;
    //   RightMotorRunning = false;
    //   LeftMotorRunning = false;
    // }
    // else
    // {
    //   CoralPossession = false;
    //   RightMotorRunning = true;
    //   LeftMotorRunning = true;
    // }
    
  }
  
  public boolean CheckRightCoral()
  {
    boolean stopMotor = false;
    if( RightMotorRunning == true )
    {
      /* Since the motor is running, check its speed. */
      if( m_rightShooterEncoder.getVelocity() > 1 )
      {
        stopMotor = false;
      }
      else
      {
        stopMotor = true;
      }
    }
    return stopMotor;
  }

  public boolean CheckLeftCoral()
  {
    boolean stopMotor = false;
    if( LeftMotorRunning == true )
    {
      /* Since the motor is running, check its speed. */
      if( m_LeftShooterEncoder.getVelocity() > 1 )
      {
        stopMotor = false;
      }
      else
      {
        stopMotor = true;
      }
    }
    return stopMotor;
  }


  public void stopCoral()
  {
    m_LeftShooterMotor.set( 0 );
    m_RightShooterMotor.set( 0 );

    //LeftMotorRunning = false;
    //RightMotorRunning = false;

    //if( CoralPossession)
    //{
    //  LED.SetAllGreenCmd();
      //CoralPossession = false;
    //}
    //else
    //{
    //  LED.SetAllRedCmd();
    //}
  }   

/****************************** Commands **************************************/

  public Command CoralRunMotorCmd( double voltage )
  {
    return new RunCommand
    ( 
      () -> this.runCoralMotor( voltage ) , 
      this 
    );
  }

  public Command CoralStopMotorCmd()
  {
    return new RunCommand
    ( 
      () -> this.stopCoral() , 
      this 
    );
  }
  
}
