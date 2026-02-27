package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeConstants.IntakeArmSetpoints;
import frc.robot.Configs;

public class IntakeSubsystem extends SubsystemBase
{
  public enum IntakeArmSetpoint
  {
    k_Stow,
    k_MiniStow,
    k_Ground,
    k_Jog;
  }

  /* Define the Motors */
  private final SparkMax m_LeftIntakeArmMotor;
  // private final SparkMax m_RightIntakeArmMotor;
  private final SparkMax m_IntakeRollerMotor;

  /* Define the relative encoders for the intake. */
  private RelativeEncoder m_LeftIntakeArmEncoder;
  // private RelativeEncoder m_RightIntakeArmEncoder;
  private RelativeEncoder m_IntakeRollerEncoder;

  /* Define Spark PID Loops */
  private SparkClosedLoopController m_IntakeArmPIDController;
  //private SparkClosedLoopController m_IntakeRollerPIDController;

  /* Setpoint Tracker for PID Loops */
  private double m_IntakeAngleSetpoint = IntakeArmSetpoints.k_Stow;
  private double m_IntakeSpeedSetpoint = 0;

  public IntakeSubsystem()
  {
    /* Setup the motors */
    m_LeftIntakeArmMotor  = new SparkMax( Constants.IntakeConstants.k_LeftIntakeArmMotorCANId, MotorType.kBrushless );
    // m_RightIntakeArmMotor = new SparkMax( Constants.IntakeConstants.k_RightIntakeArmMotorCANId, MotorType.kBrushless );
    m_IntakeRollerMotor   = new SparkMax( Constants.IntakeConstants.k_IntakeRollerMotorCANId, MotorType.kBrushless );

    /* Need to setup an encoder, dont think we will need one. */
    m_LeftIntakeArmEncoder  = m_LeftIntakeArmMotor.getEncoder();
    // m_RightIntakeArmEncoder = m_RightIntakeArmMotor.getEncoder();
    m_IntakeRollerEncoder   = m_IntakeRollerMotor.getEncoder();

    /* Setup a PID loop. */
    m_IntakeArmPIDController = m_LeftIntakeArmMotor.getClosedLoopController();

    /* Configure the Motors for use. */
    /* Configure the Left Intake Arm Motor */
    m_LeftIntakeArmMotor.configure
    (
      Configs.IntakeModule.IntakeArmMotorConfig, 
      ResetMode.kResetSafeParameters, 
      PersistMode.kPersistParameters
    );

    /* Configure the Right Intake Arm Motor */
    // m_RightIntakeArmMotor.configure
    // (
    //   /* The right motor has to follow the left, set that up. */
    //   Configs.IntakeModule.IntakeArmMotorConfig.follow
    //   ( 
    //     Constants.IntakeConstants.k_RightIntakeArmMotorCANId, 
    //     false
    //   ),
    //   ResetMode.kResetSafeParameters,
    //   PersistMode.kPersistParameters 
    // );

    /* Configure the Intake Roller Motor */
    m_IntakeRollerMotor.configure
    (
      Configs.IntakeModule.IntakeRollerMotorConfig, 
      ResetMode.kResetSafeParameters, 
      PersistMode.kPersistParameters
    );

    setDefaultCommand( setIntakeArmSetpointCmd( IntakeArmSetpoint.k_Stow ) );
  }

  /************************** Smart Dashboard Values ****************************/
@Override
  public void periodic() 
  {
    moveIntakeToSetpoint();
    
    /* Print out the Intake important data. */
    /* Left intake Motor Data. */
    SmartDashboard.putNumber( "Intake | Arm | Position:", m_LeftIntakeArmEncoder.getPosition() );
    SmartDashboard.putNumber( "Intake | Arm | Velocity:", m_LeftIntakeArmEncoder.getVelocity() );
    SmartDashboard.putNumber( "Intake | Arm | Applied Output", m_LeftIntakeArmMotor.getAppliedOutput());
    SmartDashboard.putNumber( "Intake | Arm | Current", m_LeftIntakeArmMotor.getOutputCurrent());

    /* Intake Roller Motor Data. */
    SmartDashboard.putNumber( "Intake | Roller | Position:", m_IntakeRollerEncoder.getPosition() );
    SmartDashboard.putNumber( "Intake | Roller | Velocity:", m_IntakeRollerEncoder.getVelocity() );
    SmartDashboard.putNumber( "Intake | Roller | Applied Output", m_IntakeRollerMotor.getAppliedOutput());
    SmartDashboard.putNumber( "Intake | Roller | Current", m_IntakeRollerMotor.getOutputCurrent());


  }

  /********************* Helper Functions for Intake *************************/
  public double getIntakePosition()
  {
    /* Just do left arm values as the right arm follows it. */
    return m_LeftIntakeArmEncoder.getPosition();
  }

  public double getIntakeVelocity()
  {
    /* Just do left arm values as the right arm follows it. */
    return m_LeftIntakeArmEncoder.getVelocity();
  }

  public void resetIntakePosition()
  {
    /* Just do left arm values as the right arm follows it. */
    m_LeftIntakeArmEncoder.setPosition( IntakeArmSetpoints.k_Stow );
    // m_RightIntakeArmEncoder.setPosition( IntakeArmSetpoints.k_Stow );

    

  }

  public void setIntakeVoltage( double voltage )
  {
    m_LeftIntakeArmMotor.setVoltage( voltage );
    // m_RightIntakeArmMotor.setVoltage( voltage );
  }

  public void moveIntakeToSetpoint()
  {
    m_IntakeArmPIDController.setSetpoint( m_IntakeAngleSetpoint, ControlType.kPosition );
  }





  /***************************** Commands **************************************/
  public Command setIntakeArmSetpointCmd( IntakeArmSetpoint setpoint )
  {
    return this.runOnce
    (
      () -> 
      {
        switch( setpoint )
        {
          case k_Stow:
            m_IntakeAngleSetpoint = IntakeArmSetpoints.k_Stow;
            m_IntakeSpeedSetpoint = 0;
            break;
          
          case k_MiniStow:
            m_IntakeAngleSetpoint = IntakeArmSetpoints.k_MiniStow;
            m_IntakeSpeedSetpoint = 4;
            break;

          case k_Ground:
            m_IntakeAngleSetpoint = IntakeArmSetpoints.k_Ground;
            m_IntakeSpeedSetpoint = 4;
            break;
          case k_Jog:
            m_IntakeAngleSetpoint = IntakeArmSetpoints.k_Jog;
            m_IntakeSpeedSetpoint = 4;
            break;
        }
        
      }

    );
  }

  public Command runIntakeCommand()
  {
    return Commands.startEnd
    ( 
      () -> m_IntakeRollerMotor.setVoltage( 4 ), 
      () -> m_IntakeRollerMotor.setVoltage( 0 )
    );
  }

  public Command StopIntakeCommand()
  {
    return Commands.startEnd
    ( 
      () -> m_IntakeRollerMotor.setVoltage( 4 ), 
      () -> m_IntakeRollerMotor.setVoltage( 0 )
    );
  }

  public Command revIntakeCommand()
  {
    return Commands.startEnd
    ( 
      () -> m_IntakeRollerMotor.setVoltage( -4 ), 
      () -> m_IntakeRollerMotor.setVoltage( 0 )
    );
  }

  public Command runIntakeVoltageCommand( double voltage)
  {
    return Commands.startEnd
    ( 
      () -> m_IntakeRollerMotor.setVoltage( voltage ), 
      () -> m_IntakeRollerMotor.setVoltage( 0 )
    );
  }

  public Command jogIntakeUpDownCommand() 
  {
    return Commands.sequence
    (
      setIntakeArmSetpointCmd( IntakeArmSetpoint.k_Ground ).withTimeout(0.5),
      setIntakeArmSetpointCmd( IntakeArmSetpoint.k_Jog ).withTimeout(0.5)
    )
    .repeatedly();
  }


}
