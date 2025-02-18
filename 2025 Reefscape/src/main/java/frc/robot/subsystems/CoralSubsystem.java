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


public class CoralSubsystem extends SubsystemBase
{
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
    SmartDashboard.putNumber( "LeftCoralEncoder:", m_LeftCoralEncoder.getPosition() );
    SmartDashboard.putNumber( "LeftCoralSpeed:", m_LeftCoralEncoder.getVelocity() );

    SmartDashboard.putNumber( "RightCoralEncoder:", m_rightCoralEncoder.getPosition() );
    SmartDashboard.putNumber( "RightCoralSpeed:", m_rightCoralEncoder.getVelocity() );

    

    SmartDashboard.putNumber( " LeftCoralCurrent", m_LeftCoralMotor.getOutputCurrent() );
    SmartDashboard.putNumber( "RightCoralCurrent", m_RightCoralMotor.getOutputCurrent() );
  
    
  
  
  }

/*********************** Helper Functions for Coral ***************************/



public void runCoralMotor( double voltage )
  {
    //if( !currentLimitReached()  )
    //{
      m_LeftCoralMotor.set( voltage );
      m_RightCoralMotor.set( voltage );
    //}
    //stopCoral();
  }    

  public void stopCoral()
  {
    m_LeftCoralMotor.set( 0 );
    m_RightCoralMotor.set( 0 );
  } 

  

/****************************** Commands **************************************/

  public Command CoralRunMotorCmd( double voltage)
  {
    return new RunCommand
    ( 
      () -> this.runCoralMotor( voltage ) , 
      this 
    );
  }

  
}
