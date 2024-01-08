// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoMode.PixelFormat;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.Claw1_Command;
import frc.robot.commands.GrabOpen;
import frc.robot.commands.GrabToggle;
import frc.robot.commands.elbowMotorCommand;
import frc.robot.commands.shoulderMotorCommand;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.subsystems.elbowMotorSub;
import frc.robot.subsystems.shoulderMotorSub;
import frc.robot.subsystems.Claw1_Subsystem;
import frc.robot.subsystems.Grab_Subsystem;
import frc.robot.subsystems.airBrake_Subsystem;
import frc.robot.subsystems.autoDrive_Subsystem;
import frc.robot.subsystems.autoArm_Subsystem;
import frc.robot.RobotContainer;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kForward;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kOff;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kReverse;

//import frc.robot.subsystems.Claw1_Subsystem;
//import frc.robot.subsystems.shoulderMotorSub;

/**
 * This is a demo program showing the use of the DifferentialDrive class,
 * specifically it contains
 * the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {
  // private Joystick m_leftStick;
  // private Joystick m_rightStick;

  // public static Claw1_Subsystem clawSpeed;
  // public static shoulderMotorSub shoulderSpeed;

  public static RobotContainer m_robotContainer;

  private static WPI_VictorSPX m_leftMotorA = new WPI_VictorSPX(Constants.m_leftMotorA),
      m_leftMotorB = new WPI_VictorSPX(Constants.m_leftMotorB),
      m_rightMotorA = new WPI_VictorSPX(Constants.m_rightMotorA),
      m_rightMotorB = new WPI_VictorSPX(Constants.m_rightMotorB),
      shoulderMotorA = new WPI_VictorSPX(Constants.shoulderMotorA),
      shoulderMotorB = new WPI_VictorSPX(Constants.shoulderMotorB),
      clawMotor = new WPI_VictorSPX(Constants.clawMotor);

  MotorControllerGroup m_leftMotors = new MotorControllerGroup(m_leftMotorA, m_leftMotorB);
  MotorControllerGroup m_rightMotors = new MotorControllerGroup(m_rightMotorA, m_rightMotorB);

  MotorControllerGroup elbowMotors = new MotorControllerGroup(shoulderMotorA, shoulderMotorB);

  Compressor pcmCompressor = new Compressor(0, PneumaticsModuleType.CTREPCM);

  private DifferentialDrive m_myRobot;
  private DifferentialDrive armSystem;
  private Joystick m_leftStick;
  private Joystick m_rightStick;
  boolean enabled = pcmCompressor.isEnabled();
  boolean pressureSwitch = pcmCompressor.getPressureSwitchValue();

  public static shoulderMotorSub shoulderMotorSpeed;
  public static elbowMotorSub elbowMotorSpeed;
  public static Claw1_Subsystem claw1Speed;
  public static Grab_Subsystem Grab_Subsystem = null;
  public static airBrake_Subsystem airBrake_Subsystem = null;
  public static autoDrive_Subsystem autoDrive;
  public static autoArm_Subsystem autoArm;

  Timer myAutoTimer = new Timer();

  DigitalInput armTuckLimit = new DigitalInput(Constants.clawSwitchTop);

  public boolean armTuckLimitValue() {
    return armTuckLimit.get();
  }

  CameraServer server;

  @Override
  public void robotInit() {

    m_robotContainer = new RobotContainer();

    m_myRobot = new DifferentialDrive(m_leftMotors, m_rightMotors);
    armSystem = new DifferentialDrive(clawMotor, elbowMotors);
    m_leftStick = new Joystick(0);
    m_rightStick = new Joystick(1);
    m_leftMotors.setInverted(true);

    autoDrive = new autoDrive_Subsystem();
    autoArm = new autoArm_Subsystem();
    elbowMotorSpeed = new elbowMotorSub();
    shoulderMotorSpeed = new shoulderMotorSub();
    claw1Speed = new Claw1_Subsystem();
    Grab_Subsystem = new Grab_Subsystem();
    airBrake_Subsystem = new airBrake_Subsystem();

    UsbCamera driveCamera = CameraServer.startAutomaticCapture();

    driveCamera.setVideoMode(PixelFormat.kMJPEG, 320, 240, 30);

    pcmCompressor.disable();

  }

  @Override
  public void autonomousInit() {
    CommandScheduler.getInstance().run();

    myAutoTimer.reset();
    myAutoTimer.start();
  }

  @Override

  public void autonomousPeriodic() {
    /*
     ** autoArm : +elbow turns up, +shoulder extends out.
     ** elbowMotorSpeed : + turns the winch out.
     ** Grab_Subsystem : toggles the claw
     ** autoDrive : + drives foward
     */

    if (myAutoTimer.get() < 0.5) {
      /* Turn the Arm. */
      autoArm.setAutoArm(.6, .7);
      /* Drop the winch. */
      elbowMotorSpeed.setElbowSpeed(.1);
      /* Toggle the claw. */
      // Grab_Subsystem.grabToggle();
      /* Drive the robot. */
      // autoDrive.setAutoDrive(.5);
    }
    if ((myAutoTimer.get() >= 0.5) && (myAutoTimer.get() < 1.0)) {
      /* Turn the Arm. */
      autoArm.setAutoArm(0, .8);
      /* Drop the winch. */
      elbowMotorSpeed.setElbowSpeed(.1);
      /* Toggle the claw. */
      // Grab_Subsystem.grabToggle();
      /* Drive the robot. */
      // autoDrive.setAutoDrive(.5);
    }
    if ((myAutoTimer.get() >= 1.0) && (myAutoTimer.get() < 1.5)) {
      /* Turn the Arm. */
      autoArm.setAutoArm(0, .8);
      /* Drop the winch. */
      elbowMotorSpeed.setElbowSpeed(0);
      /* Toggle the claw. */
      // Grab_Subsystem.grabToggle();
      /* Drive the robot. */
      // autoDrive.setAutoDrive(.5);
    }
    if ((myAutoTimer.get() >= 1.5) && (myAutoTimer.get() < 2.0)) {
      /* Turn the Arm. */
      autoArm.setAutoArm(-.3, .8);
      /* Drop the winch. */
      elbowMotorSpeed.setElbowSpeed(0);
      /* Toggle the claw. */
      // Grab_Subsystem.grabToggle();
      /* Drive the robot. */
      // autoDrive.setAutoDrive(.5);
    }
    if ((myAutoTimer.get() >= 2.0) && (myAutoTimer.get() < 2.5)) {
      /* Turn the Arm. */
      autoArm.setAutoArm(-.3, .7);
      /* Drop the winch. */
      // elbowMotorSpeed.setElbowSpeed(.4);
      /* Toggle the claw. */
      // Grab_Subsystem.grabToggle();
      /* Drive the robot. */
      // autoDrive.setAutoDrive(.5);
    }
    if ((myAutoTimer.get() >= 2.5) && (myAutoTimer.get() < 3.0)) {
      /* Turn the Arm. */
      autoArm.setAutoArm(0, .7);
      /* Drop the winch. */
      // elbowMotorSpeed.setElbowSpeed(.4);
      /* Toggle the claw. */
      // Grab_Subsystem.grabToggle();
      /* Drive the robot. */
      // autoDrive.setAutoDrive(.5);
    }
    if ((myAutoTimer.get() >= 3.0) && (myAutoTimer.get() < 3.5)) {
      /* Turn the Arm. */
      autoArm.setAutoArm(0, .7);
      /* Drop the winch. */
      // elbowMotorSpeed.setElbowSpeed(.4);
      /* Toggle the claw. */
      // Grab_Subsystem.grabToggle();
      /* Drive the robot. */
      // autoDrive.setAutoDrive(.5);
    }
    if ((myAutoTimer.get() >= 3.5) && (myAutoTimer.get() < 4.0)) {
      /* Turn the Arm. */
      autoArm.setAutoArm(0, .7);
      /* Drop the winch. */
      // elbowMotorSpeed.setElbowSpeed(.4);
      /* Toggle the claw. */
      // Grab_Subsystem.grabToggle();
      /* Drive the robot. */
      // autoDrive.setAutoDrive(.5);
    }
    if ((myAutoTimer.get() >= 4.0) && (myAutoTimer.get() < 4.5)) {
      /* Turn the Arm. */
      autoArm.setAutoArm(-.4, .7);
      /* Drop the winch. */
      // elbowMotorSpeed.setElbowSpeed(.4);
      /* Toggle the claw. */
      // Grab_Subsystem.grabToggle();
      /* Drive the robot. */
      // autoDrive.setAutoDrive(.5);
    }
    if ((myAutoTimer.get() >= 4.5) && (myAutoTimer.get() < 5.0)) {
      /* Turn the Arm. */
      autoArm.setAutoArm(-.4, .8);
      /* Drop the winch. */
      // elbowMotorSpeed.setElbowSpeed(.4);
      /* Toggle the claw. */
      // Grab_Subsystem.grabToggle();
      /* Drive the robot. */
      // autoDrive.setAutoDrive(.5);
    }
    if ((myAutoTimer.get() >= 5.0) && (myAutoTimer.get() < 5.5)) {
      /* Turn the Arm. */
      autoArm.setAutoArm(-.3, .8);
      /* Drop the winch. */
      // elbowMotorSpeed.setElbowSpeed(.4);
      /* Toggle the claw. */
      // Grab_Subsystem.grabToggle();
      /* Drive the robot. */
      // autoDrive.setAutoDrive(.5);
    }
    if ((myAutoTimer.get() >= 5.5) && (myAutoTimer.get() < 6.0)) {
      /* Turn the Arm. */
      autoArm.setAutoArm(-.3, .7);
      /* Drop the winch. */
      // elbowMotorSpeed.setElbowSpeed(.4);
      /* Toggle the claw. */
      // Grab_Subsystem.grabToggle();
      /* Drive the robot. */
      // autoDrive.setAutoDrive(.5);
    }
    if ((myAutoTimer.get() >= 6.0) && (myAutoTimer.get() < 6.5)) {
      /* Turn the Arm. */
      autoArm.setAutoArm(-.3, .6);
      /* Drop the winch. */
      // elbowMotorSpeed.setElbowSpeed(.4);
      /* Toggle the claw. */
      // Grab_Subsystem.grabToggle();
      /* Drive the robot. */
       autoDrive.setAutoDrive(.5);
    }
    if ((myAutoTimer.get() >= 6.5) && (myAutoTimer.get() < 7.0)) {
      /* Turn the Arm. */
      autoArm.setAutoArm(-.1, 0);
      /* Drop the winch. */
      // elbowMotorSpeed.setElbowSpeed(.4);
      /* Toggle the claw. */
      // Grab_Subsystem.grabToggle();
      /* Drive the robot. */
       autoDrive.setAutoDrive(.5);
    }
    if ((myAutoTimer.get() >= 7.0) && (myAutoTimer.get() < 7.5)) {
      /* Turn the Arm. */
       autoArm.setAutoArm(.1, 0);
      /* Drop the winch. */
      // elbowMotorSpeed.setElbowSpeed(.4);
      /* Toggle the claw. */
      // Grab_Subsystem.grabToggle();
      /* Drive the robot. */
      // autoDrive.setAutoDrive(.5);
    }
    if ((myAutoTimer.get() >= 7.5) && (myAutoTimer.get() < 8.0)) {
      /* Turn the Arm. */
       autoArm.setAutoArm(.8, 0);
      /* Drop the winch. */
      // elbowMotorSpeed.setElbowSpeed(.4);
      /* Toggle the claw. */
      // Grab_Subsystem.grabToggle();
      /* Drive the robot. */
      // autoDrive.setAutoDrive(.5);
    }
    if ((myAutoTimer.get() >= 8.0) && (myAutoTimer.get() < 8.5)) {
      /* Turn the Arm. */
       autoArm.setAutoArm(.7, 0);
      /* Drop the winch. */
      // elbowMotorSpeed.setElbowSpeed(.4);
      /* Toggle the claw. */
      // Grab_Subsystem.grabToggle();
      /* Drive the robot. */
      // autoDrive.setAutoDrive(.5);
    }
    if ((myAutoTimer.get() >= 8.5) && (myAutoTimer.get() < 8.0)) {
      /* Turn the Arm. */
       autoArm.setAutoArm(.5, 0);
      /* Drop the winch. */
      // elbowMotorSpeed.setElbowSpeed(.4);
      /* Toggle the claw. */
      // Grab_Subsystem.grabToggle();
      /* Drive the robot. */
      // autoDrive.setAutoDrive(.5);
    }
    if ((myAutoTimer.get() >= 9.0) && (myAutoTimer.get() < 9.5)) {
      /* Turn the Arm. */
       autoArm.setAutoArm(.4, 0);
      /* Drop the winch. */
      // elbowMotorSpeed.setElbowSpeed(.4);
      /* Toggle the claw. */
      // Grab_Subsystem.grabToggle();
      /* Drive the robot. */
       autoDrive.setAutoDrive(-.5);
    }
    if ((myAutoTimer.get() >= 9.5) && (myAutoTimer.get() < 10.0)) {
      /* Turn the Arm. */
      // autoArm.setAutoArm(.6, .7);
      /* Drop the winch. */
      // elbowMotorSpeed.setElbowSpeed(.4);
      /* Toggle the claw. */
       Grab_Subsystem.grabClose();
      /* Drive the robot. */
      // autoDrive.setAutoDrive(.5);
    }
    if ((myAutoTimer.get() >= 10.0) && (myAutoTimer.get() < 10.5)) {
      /* Turn the Arm. */
      // autoArm.setAutoArm(.6, .7);
      /* Drop the winch. */
      // elbowMotorSpeed.setElbowSpeed(.4);
      /* Toggle the claw. */
      // Grab_Subsystem.grabToggle();
      /* Drive the robot. */
       autoDrive.setAutoDrive(-.5);
    }
    if ((myAutoTimer.get() >= 10.5) && (myAutoTimer.get() < 11.0)) {
      /* Turn the Arm. */
      // autoArm.setAutoArm(.6, .7);
      /* Drop the winch. */
      // elbowMotorSpeed.setElbowSpeed(.4);
      /* Toggle the claw. */
      // Grab_Subsystem.grabToggle();
      /* Drive the robot. */
       autoDrive.setAutoDrive(-.5);
    }
    if ((myAutoTimer.get() >= 11.0) && (myAutoTimer.get() < 11.5)) {
      /* Turn the Arm. */
       autoArm.setAutoArm(-.6, -.7);
      /* Drop the winch. */
      // elbowMotorSpeed.setElbowSpeed(.4);
      /* Toggle the claw. */
      // Grab_Subsystem.grabToggle();
      /* Drive the robot. */
       autoDrive.setAutoDrive(-.5);
    }
    if ((myAutoTimer.get() >= 11.5) && (myAutoTimer.get() < 12.0)) {
      /* Turn the Arm. */
       autoArm.setAutoArm(-.6, -.7);
      /* Drop the winch. */
      // elbowMotorSpeed.setElbowSpeed(.4);
      /* Toggle the claw. */
      // Grab_Subsystem.grabToggle();
      /* Drive the robot. */
      // autoDrive.setAutoDrive(.5);
      autoDrive.setAutoDrive(-.6);
    }
    if ((myAutoTimer.get() >= 12.0) && (myAutoTimer.get() < 12.5)) {
      /* Turn the Arm. */
       autoArm.setAutoArm(-.6, -.7);
      /* Drop the winch. */
      // elbowMotorSpeed.setElbowSpeed(.4);
      /* Toggle the claw. */
      // Grab_Subsystem.grabToggle();
      /* Drive the robot. */
      // autoDrive.setAutoDrive(.5);
      autoDrive.setAutoDrive(-.6);
    }
    if ((myAutoTimer.get() >= 12.5) && (myAutoTimer.get() < 13.0)) {
      /* Turn the Arm. */
       autoArm.setAutoArm(-.6, -.9);
      /* Drop the winch. */
      // elbowMotorSpeed.setElbowSpeed(.4);
      /* Toggle the claw. */
      // Grab_Subsystem.grabToggle();
      /* Drive the robot. */
      // autoDrive.setAutoDrive(.5);
      autoDrive.setAutoDrive(-.4);
    }
    if ((myAutoTimer.get() >= 13.0) && (myAutoTimer.get() < 13.5)) {
      /* Turn the Arm. */
       autoArm.setAutoArm(-.6, -.9);
      /* Drop the winch. */
      // elbowMotorSpeed.setElbowSpeed(.4);
      /* Toggle the claw. */
      // Grab_Subsystem.grabToggle();
      /* Drive the robot. */
      // autoDrive.setAutoDrive(.5);
      autoDrive.setAutoDrive(-.6);
    }
    if ((myAutoTimer.get() >= 13.5) && (myAutoTimer.get() < 14.0)) {
      /* Turn the Arm. */
       autoArm.setAutoArm(-.6, -.9);
      /* Drop the winch. */
      // elbowMotorSpeed.setElbowSpeed(.4);
      /* Toggle the claw. */
      // Grab_Subsystem.grabToggle();
      /* Drive the robot. */
      // autoDrive.setAutoDrive(.5);
      autoDrive.setAutoDrive(-.6);
    }
    if ((myAutoTimer.get() >= 14.0) && (myAutoTimer.get() < 14.5)) {
      /* Turn the Arm. */
       autoArm.setAutoArm(0, -.9);
      /* Drop the winch. */
      // elbowMotorSpeed.setElbowSpeed(.4);
      /* Toggle the claw. */
      // Grab_Subsystem.grabToggle();
      /* Drive the robot. */
      // autoDrive.setAutoDrive(.5);
      autoDrive.setAutoDrive(-.6);
    }
    if ((myAutoTimer.get() >= 14.5) && (myAutoTimer.get() < 15.0)) {
      /* Turn the Arm. */
       autoArm.setAutoArm(0, -.8);
      /* Drop the winch. */
      // elbowMotorSpeed.setElbowSpeed(.4);
      /* Toggle the claw. */
      // Grab_Subsystem.grabToggle();
      /* Drive the robot. */
      // autoDrive.setAutoDrive(.5);
      autoDrive.setAutoDrive(-.5);
    }

    // shoulder out period of time
    // if( myAutoTimer.get() < 2 )
    // {
    // autoArm.setAutoArm(.6, .7);
    // //new elbowMotorCommand(.4);
    // //elbowMotorSpeed.setElbowSpeed(.4);
    // }
    // if( (myAutoTimer.get() >= 2) && ( myAutoTimer.get() < 2.5 ) )
    // {
    // //add shoulder movement?
    // //new elbowMotorCommand(.5);
    // elbowMotorSpeed.setElbowSpeed(.2);
    // autoArm.setAutoArm(-.6, .6);
    // //autoDrive.setAutoDrive(-.8);
    // }

    // // elbow down, period of time
    // if( (myAutoTimer.get() >= 2.2) && ( myAutoTimer.get() < 5 ) )
    // {
    // //add shoulder movement?
    // //new elbowMotorCommand(.5);
    // autoArm.setAutoArm(-.5, .6);
    // }
    // //shoulder up, period of time
    // if( ( myAutoTimer.get() >= 5 ) && ( myAutoTimer.get() < 7 ) )
    // {
    // autoArm.setAutoArm(0, .8);
    // }
    // //drive forward, straight, period of time, into hard stops
    // //stall shoulder motors
    // if( ( myAutoTimer.get() >= 7 ) && ( myAutoTimer.get() < 8 ) )
    // {
    // //autoArm.setAutoArm(0, .8);
    // //autoDrive.setAutoDrive(.5);
    // }
    // //spin claw down, period of time
    // if( ( myAutoTimer.get() >= 8 ) && ( myAutoTimer.get() < 11 ) )
    // {
    // autoArm.setAutoArm(.7, 0);
    // //autoDrive.setAutoDrive(.5);
    // //new Claw1_Command ( -.5 );
    // }
    // //open claw
    // if( ( myAutoTimer.get() >= 11 ) && ( myAutoTimer.get() < 12 ) )
    // {
    // //autoArm.setAutoArm(0, .8);
    // //autoDrive.setAutoDrive(.5);
    // //new Claw1_Command(.5)
    // new GrabToggle();
    // }
    // //drive reverse, period of time
    // if( ( myAutoTimer.get() >= 12 ) && ( myAutoTimer.get() < 15 ) )
    // {
    // //autoArm.setAutoArm(0, .8);
    // //autoDrive.setAutoDrive(-.6);
  }

  

  @Override

  public void teleopInit() {
    /*
     * if (m_autonomousCommand != null) {
     * m_autonomousCommand.cancel();
     * }
     */

    // driveCommand = m_robotContainer.getDriveCommand();

  }

  @Override

  public void teleopPeriodic() {
    CommandScheduler.getInstance().run();

    m_myRobot.tankDrive((.6 * m_leftStick.getY()), (.6 * m_rightStick.getY())); 
    // if (m_leftStick.getRawButtonPressed(1) == false)
    // {
    // m_myRobot.tankDrive(( .7 * m_leftStick.getY() ), ( .7 * m_rightStick.getY() )
    // );
    // }
    // else
    // {
    // m_myRobot.tankDrive(( 1 * m_leftStick.getY() ), ( 1 * m_rightStick.getY() )
    // );

    // }

    // if (armTuckLimitValue() && m_rightStick.getX() >= 0)
    // {
    // armSystem.tankDrive(( .7 * - m_leftStick.getX() ), ( .9 * m_rightStick.getX()
    // ) ); //claw, elbow
    // }
    // else
    // {
    // armSystem.tankDrive(( .7 * - m_leftStick.getX() ), ( 0 ) ); //claw, elbow
    // }

    // armSystem.tankDrive(( .7 * - m_leftStick.getX() ), ( .8 * m_rightStick.getX()
    // ) ); //claw, elbow
    if (m_leftStick.getRawButton(1) == true) {
      // armSystem.tankDrive(( .7 * - m_leftStick.getX() ), ( .8 * m_rightStick.getX()
      // ) ); //claw, elbow
      armSystem.tankDrive(0, .3);
    }
    // else if( m_rightStick.getRawButton(2) == true )
    // {
    // armSystem.tankDrive(.3,0);
    // }
    else {
      // armSystem.tankDrive(0,.3);
      armSystem.tankDrive((.7 * -m_leftStick.getX()), (.8 * m_rightStick.getX())); // claw, elbow
    }

    // if( m_rightStick.getRawButton(2) == true )
    // {
    // //armSystem.tankDrive(( .7 * - m_leftStick.getX() ), ( .8 *
    // m_rightStick.getX() ) ); //claw, elbow
    // armSystem.tankDrive(.3,0);
    // }
    // else
    // {
    // //armSystem.tankDrive(0,.3);
    // armSystem.tankDrive(( .5 * - m_leftStick.getX() ), ( .8 * m_rightStick.getX()
    // ) ); //claw, elbow
    // }

    if (pressureSwitch == false) {
      pcmCompressor.enableDigital();
    } else {
      pcmCompressor.disable();
    }

  }
}
