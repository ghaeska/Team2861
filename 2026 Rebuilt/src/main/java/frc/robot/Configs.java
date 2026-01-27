package frc.robot;

import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
//import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.SwerveConstants.ModuleConstants;
import frc.robot.Constants;

public class Configs 
{
  /* MAXSwerve Configs */
public static final class MAXSwerveModule 
  {
    public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
    public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

    static 
    {
      // Use module constants to calculate conversion factors and feed forward gain.
      double drivingFactor = ModuleConstants.kWheelDiameterMeters * Math.PI
                / ModuleConstants.kDrivingMotorReduction;
      double turningFactor = 2 * Math.PI;
      double nominalVoltage = 12.0;
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
                ;

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
                // This applies to REV Through Bore Encoder V2 (use REV_ThroughBoreEncoder for V1):
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
  /* Subsystem Configs */
    
  }

public static final class ShooterModule
  {
    public static final SparkMaxConfig ShooterSparkMaxConfig = new SparkMaxConfig();
    public static final SparkFlexConfig ShooterSparkFlexConfig = new SparkFlexConfig();

    static
    {
      /* ------------------ Shooter Motor Configs. ------------------------- */
      ShooterSparkMaxConfig.idleMode( IdleMode.kBrake );
      ShooterSparkMaxConfig.smartCurrentLimit( Constants.ShooterConstants.k_Shooter_MaxCurrent );
      ShooterSparkMaxConfig.inverted( true );

      ShooterSparkFlexConfig.idleMode( IdleMode.kBrake );
      ShooterSparkFlexConfig.smartCurrentLimit( Constants.ShooterConstants.k_Shooter_MaxCurrent );

      ShooterSparkMaxConfig
        .absoluteEncoder
        .inverted(true)
        .positionConversionFactor( 360 );
      ShooterSparkMaxConfig
        .closedLoop
        .feedbackSensor( FeedbackSensor.kAbsoluteEncoder )
        // Set up the PID values for position control
        .p( .01 ) // GTH:TODO: tune.
        .d(0.0)
        //.velocityFF(.001)
        .outputRange( -0.1,.1 );
    }
  }



  public static final class FeedModule
  {
    public static final SparkMaxConfig FeedSparkMaxConfig = new SparkMaxConfig();
    public static final SparkFlexConfig FeedSparkFlexConfig = new SparkFlexConfig();

    static
    {
      /* ------------------ Intake Feed Motor Configs. ------------------------- */
      FeedSparkMaxConfig.idleMode( IdleMode.kBrake );
      FeedSparkMaxConfig.smartCurrentLimit( Constants.FeedConstants.k_Feed_MaxCurrent );
      FeedSparkMaxConfig.inverted( true );

      FeedSparkFlexConfig.idleMode( IdleMode.kBrake );
      FeedSparkFlexConfig.smartCurrentLimit( Constants.FeedConstants.k_Feed_MaxCurrent );

      FeedSparkMaxConfig
        .absoluteEncoder
        .inverted(true)
        .positionConversionFactor( 360 );
      FeedSparkMaxConfig
        .closedLoop
        .feedbackSensor( FeedbackSensor.kAbsoluteEncoder )
        // Set up the PID values for position control
        .p( .01 ) // GTH:TODO: tune.
        .d(0.0)
        //.velocityFF(.001)
        .outputRange( -0.1,.1 );
    }
  }

   public static final class IntakeModule
  {
    public static final SparkMaxConfig IntakeMotorCfg = new SparkMaxConfig();
    
    static 
    {
      /* ------------------ Intake Motor Configs. ------------------------- */

      /* Set the Idle mode to brake so the motors dont move when powered. */
      IntakeMotorCfg.idleMode( IdleMode.kBrake );
      /* Set the Smart Current limit to 40A to prevent motor damage. */
      IntakeMotorCfg.smartCurrentLimit( Constants.IntakeConstants.k_Int_MaxCurrent );
      
      IntakeMotorCfg.inverted( true );
      
      /* Set the PID LOOP up for the Elevator. */
      IntakeMotorCfg
        .closedLoop
        .feedbackSensor( FeedbackSensor.kPrimaryEncoder )
        // Setup PID values
        .p( 0.10 ) //GTH:TODO: tune.
        .outputRange( -.8, .8 );
    }
  }
}