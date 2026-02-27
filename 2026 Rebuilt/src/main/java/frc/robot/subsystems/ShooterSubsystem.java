package frc.robot.subsystems;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import java.util.function.Supplier;

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
  private double m_ShooterSetpoint = Constants.ShooterConstants.ShooterSpeedSetpoints.k_stop;

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
        Constants.ShooterConstants.k_LeftShooterMotorCANId, 
        true
      ),
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters 
    );

    /* Set the default command to stop the shooter. */
    //setDefaultCommand( setShooterSetpointCmd( ShooterSetpoint.k_Shoot ) );

  }

    /************************** Smart Dashboard Values ****************************/
@Override
  public void periodic() 
  {
    /* Set our Shooter motor to the desired setpoint.  We do this check every 20ms. */
    moveToSetpoint();

    /* Print out the Shooter important data. */
    /* Left shooter Motor Data. */
    SmartDashboard.putNumber( "Shooter Left | Flywheel | Position:", m_LeftShooterEncoder.getPosition() );
    SmartDashboard.putNumber( "Shooter Left | Flywheel | Velocity:", m_LeftShooterEncoder.getVelocity() );
    SmartDashboard.putNumber( "Shooter Left | Flywheel | Applied Output", m_LeftShooterMotor.getAppliedOutput());
    SmartDashboard.putNumber( "Shooter Left | Flywheel | Current", m_LeftShooterMotor.getOutputCurrent());

    SmartDashboard.putNumber( "Shooter | Flywheel | Target", m_ShooterSetpoint );

    /* Right Shooter Motor Data. */
    SmartDashboard.putNumber( "Shooter Right | Flywheel | Position:", m_RightShooterEncoder.getPosition() );
    SmartDashboard.putNumber( "Shooter Right | Flywheel | Velocity:", m_RightShooterEncoder.getVelocity() );
    SmartDashboard.putNumber( "Shooter Right | Flywheel | Applied Output", m_RightShooterMotor.getAppliedOutput());
    SmartDashboard.putNumber( "Shooter Right | Flywheel | Current", m_RightShooterMotor.getOutputCurrent());


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

  // private void setFlywheelVelocity( double velocity )
  // {
  //   m_ShooterPIDController.setSetpoint(velocity, ControlType.kVelocity );
  // }

  public Command increaseFlywheelSpeedCmd()
  {
    return this.runOnce
    (
      () ->
      {
        m_ShooterSetpoint = m_ShooterSetpoint + 25;
      }
    );
  }

  public Command decreaseFlywheelSpeedCmd()
  {
    return this.runOnce
    (
      () ->
      {
        m_ShooterSetpoint = m_ShooterSetpoint - 25;
      }
    );
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
            break;
        }
        
      }

    );

  }

  /* Using Rev's system has veloicty supplied at RPM. */
  public Command spinAtVelocityCommand(Supplier<Double> goalVelocitySupplier) {
        return run
        (
          () -> 
          {
              m_ShooterSetpoint = goalVelocitySupplier.get();
              //primaryMotor.setControl(velocityRequest.withVelocity(goalVelocitySupplier.get()));
          }
        );
    }


    /* Intake Algae Command */
  public Command ShooterForwardCommand()
  {
    return new RunCommand(()->this.setVoltage(.5), this );
  }

  /* Stop Algae Command */
  public Command ShooterStopCommand()
  {
    return new RunCommand(()->this.setVoltage(0), this );
  }










}
