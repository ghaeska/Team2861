package frc.robot.subsystems;

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
import frc.robot.Constants.IntakeConstants.IntakeArmSetpoints;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterConstants.ShooterSpeedSetpoints;
import frc.robot.subsystems.IntakeSubsystem.IntakeArmSetpoint;
import frc.robot.Configs;

public class ShooterSubsystem  extends SubsystemBase
{
  public enum ShooterSetpoint
  {
    k_Shoot,
    k_Pass,
    k_Stop;
  }

  /* Define the Motors */
  private final SparkMax m_LeftShooterMotor;
  private final SparkMax m_RightShooterMotor;

  /* Define the relative encoders for the intake. */
  private RelativeEncoder m_LeftShooterEncoder;
  private RelativeEncoder m_RightShooterEncoder;

  /* Define Spark PID Loops */
  private SparkClosedLoopController m_ShooterPIDController;

  /* Setpoint Tracker for PID Loops */
  private double m_ShooterSetpoint = ShooterSpeedSetpoints.k_shoot;

  public ShooterSubsystem()
  {
    /* Setup the motors */
    m_LeftShooterMotor  = new SparkMax( Constants.ShooterConstants.k_LeftShooterMotorCANId, MotorType.kBrushless );
    m_RightShooterMotor = new SparkMax( Constants.ShooterConstants.k_RightShooterMotorCANId, MotorType.kBrushless );
    
    /* Need to setup the encoders. */
    m_LeftShooterEncoder  = m_LeftShooterMotor.getEncoder();
    m_RightShooterEncoder = m_RightShooterMotor.getEncoder();

    /* Setup a PID loop. */
    m_ShooterPIDController = m_LeftShooterMotor.getClosedLoopController();

    /* Configure the Motors for use. */
    /* Configure the Left Shooter Motor */
    m_LeftShooterMotor.configure
    (
      Configs.ShooterModule.ShooterMotorConfig, 
      ResetMode.kResetSafeParameters, 
      PersistMode.kPersistParameters
    );

    /* Configure the Right Shooter Motor */
    m_RightShooterMotor.configure
    (
      /* The right motor has to follow the left, set that up. */
      Configs.ShooterModule.ShooterMotorConfig.follow
      ( 
        Constants.ShooterConstants.k_RightShooterMotorCANId, 
        true
      ),
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters 
    );

    /* Set the default command to stop the shooter. */
    setDefaultCommand( setShooterSetpointCmd( ShooterSetpoint.k_Stop ) );

  }

    /************************** Smart Dashboard Values ****************************/
@Override
  public void periodic() 
  {
    moveToSetpoint();

    /* Print out the Algae Encoder positions and velocities */
    SmartDashboard.putNumber( "LeftShooterEncoder:", m_LeftShooterEncoder.getPosition() );
    SmartDashboard.putNumber( "LeftShooterSpeed:", m_LeftShooterEncoder.getVelocity() );

    SmartDashboard.putNumber( "RightShooterEncoder:", m_RightShooterEncoder.getPosition() );
    SmartDashboard.putNumber( "RightShooterSpeed:", m_RightShooterEncoder.getVelocity() );

    

  }

/********************* Helper Functions for Shooter *************************/
  public double getPosition()
  {
    /* Just do left arm values as the right arm follows it. */
    return m_LeftShooterEncoder.getPosition();
  }

  public double getVelocity()
  {
    /* Just do left arm values as the right arm follows it. */
    return m_LeftShooterEncoder.getVelocity();
  }

  

  public void setVoltage( double voltage )
  {
    m_LeftShooterMotor.setVoltage( voltage );
    m_RightShooterMotor.setVoltage( voltage );
  }

  public void moveToSetpoint()
  {
    m_ShooterPIDController.setSetpoint( m_ShooterSetpoint, ControlType.kVelocity );
  }


 /***************************** Commands **************************************/
  public Command setShooterSetpointCmd( ShooterSetpoint setpoint )
  {
    return this.runOnce
    (
      () -> 
      {
        switch( setpoint )
        {
          case k_Pass:
            m_ShooterSetpoint = ShooterSpeedSetpoints.k_pass;
            break;
          
          case k_Shoot:
            m_ShooterSetpoint = ShooterSpeedSetpoints.k_shoot;
            break;
          case k_Stop:
            m_ShooterSetpoint = ShooterSpeedSetpoints.k_stop;

        }
        
      }

    );

  }









}
