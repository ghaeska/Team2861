package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import java.util.function.Supplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants.IntakeArmSetpoints;
import frc.robot.Constants.ShooterConstants.ShooterSpeedSetpoints;
import frc.robot.Constants.ShooterHoodConstants;
import frc.robot.Constants.ShooterHoodConstants.ShooterHoodSetpoints;
import frc.robot.subsystems.IntakeSubsystem.IntakeArmSetpoint;
import frc.robot.Configs;

public class ShooterHoodSubsystem  extends SubsystemBase 
{
  public enum ShooterHoodSetpoint
  {
    k_HoodMax,
    k_HoodMin;
  }
  /* Define the Motors */
  private final SparkMax m_ShooterHoodMotor;

  /* Define the relative encoders for the Hood. */
  private RelativeEncoder m_ShooterHoodEncoder;

  /* Define the absolute encoder for the hood */
  private AbsoluteEncoder m_HoodAngleEncoder;

  /* Define Spark PID Loops */
  private SparkClosedLoopController m_ShooterHoodPIDController;

  /* Setpoint Tracker for PID Loops */
  private double m_ShooterHoodSetpoint = ShooterHoodSetpoints.k_HoodMin;

  public ShooterHoodSubsystem()
  {
    /* Setup the motors */
    m_ShooterHoodMotor  = new SparkMax( Constants.ShooterHoodConstants.k_ShooterHoodMotorCANId, MotorType.kBrushless );
    
    /* Need to setup the encoders. */
    m_ShooterHoodEncoder  = m_ShooterHoodMotor.getEncoder();

    m_HoodAngleEncoder = m_ShooterHoodMotor.getAbsoluteEncoder();

    /* Setup a PID loop. */
    m_ShooterHoodPIDController = m_ShooterHoodMotor.getClosedLoopController();

    /* Configure the Motors for use. */
    m_ShooterHoodMotor.configure
    (
      Configs.ShooterHoodModule.ShooterHoodMotorConfig, 
      ResetMode.kResetSafeParameters, 
      PersistMode.kPersistParameters
    );


  }

/************************** Smart Dashboard Values ****************************/
@Override
  public void periodic() 
  {
    moveHoodToSetpoint();

    /* Print out the Shooter Hood important data. */
    /* Shooter Hood Motor Data. */
    SmartDashboard.putNumber( "Shooter Hood | Motor | Position:", m_ShooterHoodEncoder.getPosition() );
    SmartDashboard.putNumber( "Shooter Hood | Motor | Velocity:", m_ShooterHoodEncoder.getVelocity() );
    SmartDashboard.putNumber( "Shooter Hood | Motor | Applied Output", m_ShooterHoodMotor.getAppliedOutput());
    SmartDashboard.putNumber( "Shooter Hood | Motor | Current", m_ShooterHoodMotor.getOutputCurrent());

    SmartDashboard.putNumber( "Shooter Hood | ABS Encoder | Target Setpoint:", m_ShooterHoodSetpoint );
    SmartDashboard.putNumber( "Shooter Hood | ABS Encoder | Actual Setpoint:", m_HoodAngleEncoder.getPosition() );

  }

  private void moveHoodToSetpoint()
  {
    m_ShooterHoodPIDController.setSetpoint( m_ShooterHoodSetpoint, ControlType.kPosition );
  }

/***************************** Commands **************************************/
  public Command setShooterHoodSetpointCmd( ShooterHoodSetpoint setpoint )
  {
    return this.runOnce
    (
      () -> 
      {
        switch( setpoint )
        {
          case k_HoodMin:
            m_ShooterHoodSetpoint = ShooterHoodSetpoints.k_HoodMin;
            break;
          
          case k_HoodMax:
            m_ShooterHoodSetpoint = ShooterHoodSetpoints.k_HoodMax;
            break;
        }        
      }
    );
  }

  public Command moveToSpecificPositionCmd( Supplier<Double> goalPositionSupplier )
  {
    return this.runOnce
    (
      () -> 
      {
        m_ShooterHoodSetpoint = goalPositionSupplier.get();
        //m_ShooterHoodSetpoint = setpoint;
      }

    );
  }

}
