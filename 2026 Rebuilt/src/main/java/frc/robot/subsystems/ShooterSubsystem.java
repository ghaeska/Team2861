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
import frc.robot.Constants.ShooterConstants.ShooterSetpoints;
import frc.robot.Configs;



public class ShooterSubsystem extends SubsystemBase
{
   public enum ShooterSetpoint
  {
    k_shoot,
    k_pass;
  }

  /* Define the motors */
  private final SparkFlex m_LeftShooterMotor;
  private final SparkFlex m_RightShooterMotor;
  //private final SparkMax m_PivotCoralMotor;

  /* Define spark pid controller */
  private SparkClosedLoopController m_LeftShooterPIDController;
  //private SparkClosedLoopController m_RightShooterPIDController;


  /* Define Relative motor Encoders */
  private RelativeEncoder m_LeftShooterEncoder;
  private RelativeEncoder m_RightShooterEncoder;

  private double m_ShooterSetpoint = Constants.ShooterConstants.ShooterSetpoints.k_shoot;
  

  

  public ShooterSubsystem()
  {
    /* Assign the intake motor, they are spark Flex motors */
    m_LeftShooterMotor = new SparkFlex(Constants.ShooterConstants.k_LeftShooterMotorCANId, MotorType.kBrushless );
    m_RightShooterMotor = new SparkFlex(Constants.ShooterConstants.k_RightShooterMotorCANId, MotorType.kBrushless );

    
    /* Setup the motor Encoders. */
    m_LeftShooterEncoder = m_LeftShooterMotor.getEncoder();
    m_RightShooterEncoder = m_RightShooterMotor.getEncoder();

    /* Configure the left motor */
    m_LeftShooterMotor.configure
    (
      Configs.ShooterModule.ShooterSparkMaxConfig, 
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters 
    );
    /* Configure the right motor */
    m_RightShooterMotor.configure
    (
      Configs.ShooterModule.ShooterSparkMaxConfig.follow
      (Constants.ShooterConstants.k_LeftShooterMotorCANId, 
        true),
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters 
    );

   

  }

/************************** Smart Dashboard Values ****************************/
@Override
  public void periodic() 
  {
    /* Print out the Shooter Encoder positions and velocities */
    SmartDashboard.putNumber( "LeftShooterSpeed:", m_LeftShooterEncoder.getVelocity() );
    SmartDashboard.putNumber( "RightShooterSpeed:", m_RightShooterEncoder.getVelocity() );    
  
  }

/*********************** Helper Functions for Shooter ***************************/
  
public void setVoltage(double voltage)
  {
    m_LeftShooterMotor.setVoltage( voltage );
    m_RightShooterMotor.setVoltage( voltage );
  }
  
     private void moveToSetpoint()
    {
     m_LeftShooterPIDController.setReference( m_ShooterSetpoint, ControlType.kPosition );
     
    }    
  

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
    

  
/*   public boolean CheckRightCoral()
  {
    boolean stopMotor = false;
    if( RightMotorRunning == true )
    {
      /* Since the motor is running, check its speed. 
      if( m_RightShooterEncoder.getVelocity() > 1 )
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
      // Since the motor is running, check its speed. 
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
*/

  public void stopShooter()
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

 public Command setShooterSetpointCmd( ShooterSetpoint setpoint )
  {
    return this.runOnce
    (
      () -> 
      {
        switch( setpoint )
        {
          case k_shoot:
            m_ShooterSetpoint = ShooterSetpoints.k_shoot;
            break;
            case k_pass:
            m_ShooterSetpoint = ShooterSetpoints.k_pass;
            break;
        }
      }
    );
  }
}
