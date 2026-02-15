package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.SwerveConstants.OIConstants;
import edu.wpi.first.math.MathUtil;

import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.FeedConstants.FeedIntSetpoints;
import frc.robot.Constants.IntakeConstants.IntakeSetpoints;
import frc.robot.Constants.FeedConstants;
import frc.robot.Configs;

public class IntakeSubsystem extends SubsystemBase
{
  public enum IntakeSetpoint
  {
    k_Stow,
    k_MiniStow,
    k_Ground;
  }

  //Define the Motors
  private final SparkMax m_LeftIntMotor;
  private final SparkMax m_RightIntMotor;    
  private final SparkMax m_FeedIntMotor; 

  //Define spark pid controller.
  private SparkClosedLoopController m_LeftIntPIDController;
  private SparkClosedLoopController m_FeedIntPIDController;

  //Define a relative encoder for both elevator motors
  private RelativeEncoder m_LeftIntEncoder;
  private RelativeEncoder m_RightIntEncoder;
  private RelativeEncoder m_FeedIntEncoder;

  private AbsoluteEncoder m_AbsoluteEncoder;

  /*Define Spark PID Loops */
  private SparkClosedLoopController m_IntPIDController;


  private double m_IntakeSetpoint = Constants.IntakeConstants.IntakeSetpoints.k_Stow;
  private double m_FeedIntSetpoint = Constants.FeedConstants.FeedIntSetpoints.k_Stow;

  public IntakeSubsystem()
  {
    /* Assign the CAN Id's to the motors for the elevator. */
    m_LeftIntMotor  = new SparkMax( Constants.IntakeConstants.k_LeftIntakeMotorCANId, MotorType.kBrushless );
    m_RightIntMotor = new SparkMax( Constants.IntakeConstants.k_RightIntakeMotorCANId, MotorType.kBrushless );
    m_FeedIntMotor = new SparkMax( Constants.FeedConstants.k_FeedIntMotorCANId, MotorType.kBrushless );

    /* Setup the Intake Encoder. */
    //m_LeftIntEncoder = m_LeftIntMotor.getEncoder();
    //m_RightIntEncoder = m_RightIntMotor.getEncoder();
    m_FeedIntEncoder = m_FeedIntMotor.getEncoder();

 //   m_AbsoluteEncoder = m_FeedIntMotor.getAbsoluteEncoder();

    /* Setup the Elevator PID Loop. */
    m_LeftIntPIDController = m_LeftIntMotor.getClosedLoopController();
    m_FeedIntPIDController = m_FeedIntMotor.getClosedLoopController();

    /* Configure the left elevator motor from the configs. */
    m_LeftIntMotor.configure
    (
      Configs.IntakeModule.IntakeMotorCfg,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters 
    );

    /* Configure the right intake motor from the configs. */
    m_RightIntMotor.configure
    (
      /* The right motor has to follow the left, set that up. */
      Configs.IntakeModule.IntakeMotorCfg.follow
      ( 
        Constants.IntakeConstants.k_LeftIntakeMotorCANId, 
        false
      ),
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters 
    );

    /* Configure the Pivot motor. */
    m_FeedIntMotor.configure
    (
      Configs.FeedModule.FeedSparkMaxConfig, 
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters 
    );

    /* Zero out the encoders at startup. */
    m_LeftIntEncoder.setPosition( 0 );
    m_RightIntEncoder.setPosition( 0 );
    //m_FeedIntEncoder.setPosition( 0 );


  }
/************************** Smart Dashboard Values ****************************/
@Override
  public void periodic() 
  {
    moveToSetpoint();

    /* Print out the Intake Encoder positions. */
    SmartDashboard.putNumber( "RightIntakePosition:", m_RightIntEncoder.getPosition() );
    SmartDashboard.putNumber( "LeftIntakePosition:", m_LeftIntEncoder.getPosition() );
    //SmartDashboard.putNumber( "CoralPivotPosition:", m_FeedIntEncoder.getPosition() );

    SmartDashboard.putNumber( "Target Feed Position:", m_FeedIntSetpoint );
    SmartDashboard.putNumber( "Target Intake Position:", m_IntakeSetpoint );

    SmartDashboard.putNumber( "Absolute Encoder Position", m_AbsoluteEncoder.getPosition() );
  }




/********************* Helper Functions for Intake **************************/
  private void moveToSetpoint()
  {
    m_LeftIntPIDController.setReference( m_IntakeSetpoint, ControlType.kPosition );
    m_FeedIntPIDController.setReference( m_FeedIntSetpoint, ControlType.kPosition );
  }    
   
  public void setVoltage( double voltage)
    {
      m_LeftIntMotor.setVoltage( voltage );
      m_RightIntMotor.setVoltage( voltage );
    }

  /***************************** Commands **************************************/
  public Command setIntakeSetpointCmd( IntakeSetpoint setpoint )
  {
    return this.runOnce
    (
      () -> 
      {
        switch( setpoint )
        {
          case k_Stow:
            m_IntakeSetpoint = IntakeSetpoints.k_Stow;
            break;
          case k_MiniStow:
            m_IntakeSetpoint = IntakeSetpoints.k_MiniStow;
            break;
          case k_Ground:
            m_IntakeSetpoint = IntakeSetpoints.k_Ground;
            break;
          
        }
      }
    );
  }







  

  

  
}
