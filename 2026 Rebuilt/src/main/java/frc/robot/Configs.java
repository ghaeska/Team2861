package frc.robot;

import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.SwerveConstants.ModuleConstants;
import frc.robot.Constants;

public class Configs 
{
  private static final double nominalVoltage = 12.0; // Volts

  /* MAXSwerve Configs */
  public static final class MAXSwerveModule 
  {
    public static final SparkFlexConfig drivingConfig = new SparkFlexConfig();
    public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

    static 
    {
      // Use module constants to calculate conversion factors and feed forward gain.
      double drivingFactor = ModuleConstants.kWheelDiameterMeters * Math.PI
                / ModuleConstants.kDrivingMotorReduction;
      double turningFactor = 2 * Math.PI;
      double drivingVelocityFeedForward = nominalVoltage / ModuleConstants.kDriveWheelFreeSpeedRps;
      
      /* Driving Motor Configs. */
      drivingConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(50);
      drivingConfig.encoder
                .positionConversionFactor(drivingFactor) // meters
                .velocityConversionFactor(drivingFactor / 60.0); // meters per second
      drivingConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                // These are example gains you may need to them for your own robot!
                .pid(0.04, 0, 0)
                .outputRange(-1, 1)
                .feedForward.kV(drivingVelocityFeedForward);

      /* Turning Motor Configs. */
      turningConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(20);
      turningConfig.absoluteEncoder
                // Invert the turning encoder, since the output shaft rotates in the opposite
                // direction of the steering motor in the MAXSwerve Module.
                .inverted(true)
                .positionConversionFactor(turningFactor) // radians
                .velocityConversionFactor(turningFactor / 60.0) // radians per second
                .apply(AbsoluteEncoderConfig.Presets.REV_ThroughBoreEncoder);
      turningConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                // These are example gains you may need to them for your own robot!
                .pid(1, 0, 0)
                .outputRange(-1, 1)
                // Enable PID wrap around for the turning motor. This will allow the PID
                // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
                // to 10 degrees will go through 0 rather than the other direction which is a
                // longer route.
                .positionWrappingEnabled(true)
                .positionWrappingInputRange(0, turningFactor);
    }
  }

  /* Subsystem Configs */
  public static final class IntakeModule
  {
    public static final SparkMaxConfig IntakeArmMotorConfig = new SparkMaxConfig();
    public static final SparkMaxConfig IntakeRollerMotorConfig = new SparkMaxConfig();
    static
    {
      /* ------------------ Intake Arm Motor Configs. ------------------------- */
      /* Set the idle mode to coast, so it gives if we hit the intake. */
      IntakeArmMotorConfig.idleMode( IdleMode.kCoast );
      /* set the smart current limit to 30A to prevent motor damage */
      IntakeArmMotorConfig.smartCurrentLimit( Constants.IntakeConstants.k_IntakeArm_MaxCurrent );

       /* ------------------ Intake Roller Motor Configs. ------------------------- */
       /* Set the idle mode to Brake, so it can hold a ball in place. */
      IntakeRollerMotorConfig.idleMode( IdleMode.kBrake );
      /* set the smart current limit to 30A to prevent motor damage */
      IntakeRollerMotorConfig.smartCurrentLimit( Constants.IntakeConstants.k_IntakeRoller_MaxCurrent );
    
      /* this was taken from last years elevator. */
      IntakeArmMotorConfig
        .closedLoop
        .feedbackSensor( FeedbackSensor.kPrimaryEncoder )
        .p( 0.1 )
        .outputRange( -0.2, 0.2 );    
    }
  }

  public static final class HopperModule
  {
    public static final SparkMaxConfig HopperMotorConfig = new SparkMaxConfig();
    static
    {
      /* ------------------ Hopper Motor Configs. ------------------------- */
      /* Set the idle mode to brake, the wheels and rollers stop right away . */
      HopperMotorConfig.idleMode( IdleMode.kBrake );
      /* set the smart current limit to 40A to prevent motor damage */
      HopperMotorConfig.smartCurrentLimit( Constants.HopperConstants.k_Hopper_MaxCurrent );
    }
  }

  public static final class ShooterHoodModule
  {
    public static final SparkMaxConfig ShooterHoodMotorConfig = new SparkMaxConfig();
    static
    {
      /* ------------------ Shooter Hood Motor Configs. ------------------------- */
      /* Set the idle mode to brake, try to hold the position. */
      ShooterHoodMotorConfig.idleMode( IdleMode.kBrake );
      /* set the smart current limit to 40A to prevent motor damage */
      ShooterHoodMotorConfig.smartCurrentLimit( Constants.ShooterHoodConstants.k_ShooterHood_MaxCurrent );
    
      /* Comment. */
      ShooterHoodMotorConfig
        .absoluteEncoder
        .inverted( false )
        .positionConversionFactor( 360 );
      ShooterHoodMotorConfig
        .closedLoop
        .feedbackSensor( FeedbackSensor.kAbsoluteEncoder )
        .p( 0.0001 )
        .outputRange( -.1, .1 );    
    }
  }

  public static final class ShooterModule
  {
    public static final SparkMaxConfig ShooterMotorConfig = new SparkMaxConfig();
    static
    {
      /* ------------------ Shooter Motor Configs. ------------------------- */

      double ShooterVelocityFeedForward = nominalVoltage / Constants.ShooterConstants.k_ShooterFreeSpeedRps;
      
      /* Set the idle mode to coast, the wheels dont need to slow all the way down. */
      ShooterMotorConfig.idleMode( IdleMode.kCoast );
      /* set the smart current limit to 40A to prevent motor damage */
      ShooterMotorConfig.smartCurrentLimit( Constants.ShooterConstants.k_Shooter_MaxCurrent );
      ShooterMotorConfig.inverted(true);
      ShooterMotorConfig.voltageCompensation( nominalVoltage );

      /* Uncomment this to see if it will fix the issue. */
      ShooterMotorConfig
        .encoder
          .positionConversionFactor( 1 )
          .velocityConversionFactor( 1 );
    
      
      /* this was taken from last years elevator. */
      ShooterMotorConfig
        .closedLoop
        .feedbackSensor( FeedbackSensor.kPrimaryEncoder )
        .p( 0.0005 )
        .i( 0 )
        .d( 0 )
        .outputRange( -1, 1 );

      ShooterMotorConfig
        .closedLoop
          .feedForward
            //.kV( nominalVoltage / Constants.Motors.Neo2_0MotorConstants.k_NeoKv );
            .kV( ShooterVelocityFeedForward );
    }
  }
    
}
