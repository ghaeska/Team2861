package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.Command;
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
    k_Ground;
  }

  /* Define the Motors */
  private final SparkMax m_LeftIntakeArmMotor;
  private final SparkMax m_RightIntakeArmMotor;
  private final SparkMax m_IntakeRollerMotor;

  /* Define the relative encoders for the intake. */
  private RelativeEncoder m_LeftIntakeArmEncoder;
  private RelativeEncoder m_RightIntakeArmEncoder;
  private RelativeEncoder m_IntakeRollerEncoder;

  /* Define Spark PID Loops */
  private SparkClosedLoopController m_IntakeArmPIDController;
  //private SparkClosedLoopController m_IntakeRollerPIDController;

  /* Setpoint Tracker for PID Loops */
  private double m_IntakeSetpoint = IntakeArmSetpoints.k_Stow;

  public IntakeSubsystem()
  {
    /* Setup the motors */
    m_LeftIntakeArmMotor  = new SparkMax( Constants.IntakeConstants.k_LeftIntakeArmMotorCANId, MotorType.kBrushless );
    m_RightIntakeArmMotor = new SparkMax( Constants.IntakeConstants.k_RightIntakeArmMotorCANId, MotorType.kBrushless );
    m_IntakeRollerMotor   = new SparkMax( Constants.IntakeConstants.k_IntakeRollerMotorCANId, MotorType.kBrushless );

    /* Need to setup an encoder, dont think we will need one. */
    m_LeftIntakeArmEncoder  = m_LeftIntakeArmMotor.getEncoder();
    m_RightIntakeArmEncoder = m_RightIntakeArmMotor.getEncoder();
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
    m_RightIntakeArmMotor.configure
    (
      /* The right motor has to follow the left, set that up. */
      Configs.IntakeModule.IntakeArmMotorConfig.follow
      ( 
        Constants.IntakeConstants.k_RightIntakeArmMotorCANId, 
        false
      ),
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters 
    );

    /* Configure the Intake Roller Motor */
    m_IntakeRollerMotor.configure
    (
      Configs.IntakeModule.IntakeRollerMotorConfig, 
      ResetMode.kResetSafeParameters, 
      PersistMode.kPersistParameters
    );
  }

  /************************** Smart Dashboard Values ****************************/
@Override
  public void periodic() 
  {
    moveToSetpoint();

    /* Print out the Algae Encoder positions and velocities */
    SmartDashboard.putNumber( "LeftIntakeArmEncoder:", m_LeftIntakeArmEncoder.getPosition() );
    SmartDashboard.putNumber( "LeftIntakeArmSpeed:", m_LeftIntakeArmEncoder.getVelocity() );

    SmartDashboard.putNumber( "RightIntakeArmEncoder:", m_RightIntakeArmEncoder.getPosition() );
    SmartDashboard.putNumber( "RightIntakeArmSpeed:", m_RightIntakeArmEncoder.getVelocity() );

    SmartDashboard.putNumber( "IntakeRollerEncoder:", m_IntakeRollerEncoder.getPosition() );
    SmartDashboard.putNumber( "IntakeRollerSpeed:", m_IntakeRollerEncoder.getVelocity() );

  }

  /********************* Helper Functions for Intake *************************/
  public double getPosition()
  {
    /* Just do left arm values as the right arm follows it. */
    return m_LeftIntakeArmEncoder.getPosition();
  }

  public double getVelocity()
  {
    /* Just do left arm values as the right arm follows it. */
    return m_LeftIntakeArmEncoder.getVelocity();
  }

  public void resetPosition()
  {
    /* Just do left arm values as the right arm follows it. */
    m_LeftIntakeArmEncoder.setPosition( IntakeArmSetpoints.k_Stow );
    m_RightIntakeArmEncoder.setPosition( IntakeArmSetpoints.k_Stow );

    

  }

  public void setVoltage( double voltage )
  {
    m_LeftIntakeArmMotor.setVoltage( voltage );
    m_RightIntakeArmMotor.setVoltage( voltage );
  }

  public void moveToSetpoint()
  {
    m_IntakeArmPIDController.setSetpoint( m_IntakeSetpoint, ControlType.kPosition );
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
            m_IntakeSetpoint = IntakeArmSetpoints.k_Stow;
            break;
          
          case k_MiniStow:
            m_IntakeSetpoint = IntakeArmSetpoints.k_MiniStow;
            break;

          case k_Ground:
            m_IntakeSetpoint = IntakeArmSetpoints.k_Ground;
            break;
        }
        
      }

    );
  }





}
