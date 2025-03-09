package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;

import com.revrobotics.RelativeEncoder;

public class ClimberSubsystem extends SubsystemBase
{
  /* Define Motor */
  private final SparkMax m_ClimbMotor;

  private final RelativeEncoder m_ClimbEncoder;
  
  public ClimberSubsystem()
  {
    /* Assign the climb motor the CAN ID and motor type */
    m_ClimbMotor = new SparkMax( Constants.ClimbConstants.k_ClimbMotorCANId, MotorType.kBrushless );

    m_ClimbEncoder = m_ClimbMotor.getEncoder();

    m_ClimbMotor.configure
    (
      Configs.ClimbModule.ClimbMotorConfig,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters 
    );

  }


/************************** Smart Dashboard Values ****************************/
@Override
  public void periodic() 
  {
    /* Print out the Climb motor Encoder positions and velocities */
    SmartDashboard.putNumber("Climb Encoder:", m_ClimbEncoder.getPosition() );
    SmartDashboard.putNumber("Climb Speed:", m_ClimbEncoder.getVelocity() );

  }

  public void runClimber( double voltage )
  {
    m_ClimbMotor.set( voltage );
  }

  public void stopClimber()
  {
    m_ClimbMotor.set(( 0 ));
  }

  /***************************** Commands **************************************/
  public Command ClimbCommand(double speed)
  {
    return new RunCommand( ()->this.runClimber( speed ), this );
  }

  public Command UnclimbCommand( double speed)
  {
    return new RunCommand( ()->this.runClimber( -speed ), this );
  }


}
