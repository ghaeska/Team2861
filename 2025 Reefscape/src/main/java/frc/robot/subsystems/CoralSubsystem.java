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

import frc.robot.Constants.CoralConstants;
import frc.robot.Configs;
import frc.robot.subsystems.LEDsSubsystem;


public class CoralSubsystem extends SubsystemBase
{
  private boolean RightMotorRunning = false;
  private boolean LeftMotorRunning = false;

  public boolean CoralPossession = false;

  /* Define the motors */
  private final SparkFlex m_LeftCoralMotor;
  private final SparkFlex m_RightCoralMotor;
  //private final SparkMax m_PivotCoralMotor;

  

  /* Define Relative motor Encoders */
  private RelativeEncoder m_LeftCoralEncoder;
  private RelativeEncoder m_rightCoralEncoder;

  

  public CoralSubsystem()
  {
    /* Assign the intake motor, they are spark Flex motors */
    m_LeftCoralMotor = new SparkFlex(Constants.CoralConstants.k_LeftCoralMotorCANId, MotorType.kBrushless );
    m_RightCoralMotor = new SparkFlex(Constants.CoralConstants.k_RightCoralMotorCANId, MotorType.kBrushless );

    
    /* Setup the motor Encoders. */
    m_LeftCoralEncoder = m_LeftCoralMotor.getEncoder();
    m_rightCoralEncoder = m_RightCoralMotor.getEncoder();

    /* Configure the left motor */
    m_LeftCoralMotor.configure
    (
      Configs.CoralModule.CoralSparkFlexConfig, 
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters 
    );
    /* Configure the right motor */
    m_RightCoralMotor.configure
    (
      Configs.CoralModule.CoralSparkFlexConfig
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
    SmartDashboard.putNumber( "LeftCoralSpeed:", m_LeftCoralEncoder.getVelocity() );
    SmartDashboard.putNumber( "RightCoralSpeed:", m_rightCoralEncoder.getVelocity() );    

    SmartDashboard.putNumber( " LeftCoralCurrent", m_LeftCoralMotor.getOutputCurrent() );
    SmartDashboard.putNumber( "RightCoralCurrent", m_RightCoralMotor.getOutputCurrent() );

    SmartDashboard.putBoolean( "IsLeftMotorRunning?", LeftMotorRunning );
    SmartDashboard.putBoolean( "IsRightMotorRunning?", RightMotorRunning );

    SmartDashboard.putBoolean( "Coral Possession", CoralPossession );  
  
  }

/*********************** Helper Functions for Coral ***************************/
  public void runCoralMotor( double voltage, LEDsSubsystem LED)
  {    
    m_LeftCoralMotor.set( voltage );
    m_RightCoralMotor.set( voltage );
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
      if( m_rightCoralEncoder.getVelocity() > 1 )
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
      if( m_LeftCoralEncoder.getVelocity() > 1 )
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


  public void stopCoral( LEDsSubsystem LED)
  {
    m_LeftCoralMotor.set( 0 );
    m_RightCoralMotor.set( 0 );

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

  public Command CoralRunMotorCmd( double voltage, LEDsSubsystem LED)
  {
    return new RunCommand
    ( 
      () -> this.runCoralMotor( voltage, LED ) , 
      this 
    );
  }

  public Command CoralStopMotorCmd( LEDsSubsystem LED )
  {
    return new RunCommand
    ( 
      () -> this.stopCoral( LED ) , 
      this 
    );
  }
  
}
