package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kForward;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kOff;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kReverse;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.robot.commands.Claw1_Command;
import frc.robot.commands.GrabOpen;
import frc.robot.commands.GrabToggle;
import frc.robot.commands.AirBrakeOn;
import frc.robot.commands.elbowMotorCommand;
import frc.robot.commands.shoulderMotorCommand;


public class RobotContainer 
{
    public Joystick leftJoy = new Joystick(0);
    public Joystick rightJoy = new Joystick(1);

    

    public Joystick getLeftJoystick() { return leftJoy; } 
    public Joystick getRightJoystick() { return rightJoy; }
 
   Trigger leftButton1 = new JoystickButton(leftJoy, 1);
   Trigger rightButton1 = new JoystickButton(rightJoy, 1);
   
   
   
    Trigger leftButton2 = new JoystickButton(leftJoy, 2);
    Trigger leftButton3 = new JoystickButton(leftJoy, 3);

    Trigger leftButton4 = new JoystickButton (leftJoy, 4);
    Trigger leftButton5 = new JoystickButton(leftJoy, 5); 
    Trigger leftButton6 = new JoystickButton(leftJoy, 6); 
    Trigger leftButton7 = new JoystickButton(leftJoy, 7); 

    Trigger leftButton10 = new JoystickButton(leftJoy, 10); 
    Trigger leftButton11 = new JoystickButton(leftJoy, 11); 

    Trigger rightButton2 = new JoystickButton(rightJoy, 2); 
    Trigger rightButton3 = new JoystickButton(rightJoy, 3); 

    Trigger rightButton4 = new JoystickButton(rightJoy, 4); 
    Trigger rightButton5 = new JoystickButton(rightJoy, 5); 

    Trigger rightButton10 = new JoystickButton(rightJoy, 10);
    Trigger rightButton11 = new JoystickButton(rightJoy, 11);

    Trigger rightButton6 = new JoystickButton(rightJoy, 6);
    Trigger rightButton7 = new JoystickButton(rightJoy, 7);
    Trigger rightButton8 = new JoystickButton(rightJoy, 8); 

   

    
   // DoubleSolenoid sole = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);

    
public RobotContainer()
    {
        
     
        rightButton1.whileTrue(new GrabToggle());

       //rightButton2.whileTrue(new ShoulderDown());
       //rightButton3.whileTrue(new ShoulderUp());

       rightButton4.whileTrue(new elbowMotorCommand(.3)); //Shoulder
       rightButton4.whileFalse(new elbowMotorCommand(-.1)); //Shoulder

       rightButton6.whileTrue(new elbowMotorCommand(-.3)); //Shoulder
       rightButton6.whileFalse(new elbowMotorCommand(-.1)); //Shoulder

       leftButton3.whileTrue(new elbowMotorCommand(.3)); //Shoulder
       leftButton3.whileFalse(new elbowMotorCommand(-.1)); //Shoulder

       leftButton5.whileTrue(new elbowMotorCommand(-.3)); //Shoulder
       leftButton5.whileFalse(new elbowMotorCommand(-.1)); //Shoulder
       
       rightButton11.whileTrue(new AirBrakeOn());

        //leftButton11.whileTrue(new shoulderMotorCommand(rightJoy.getRawAxis(1)));  // needed to assign a button that isn't used and is always 
        //leftButton11.whileTrue(new Claw1_Command( -leftJoy.getRawAxis(1)));         // false in order to always check for the command
        
        // rightButton4.whileTrue(new shoulderMotorCommand(-.5)); //Controls Elbow Joint
        // rightButton4.whileFalse(new shoulderMotorCommand(0)); //Controls Elbow Joint

        // rightButton5.whileTrue(new shoulderMotorCommand(.5)); //Controls Elbow Joint
        // rightButton5.whileFalse(new shoulderMotorCommand(0)); //Controls Elbow Joint

        
        

        // leftButton4.whileTrue(new Claw1_Command(-.5));
        // leftButton4.whileFalse(new Claw1_Command(0));

        // leftButton5.whileTrue(new Claw1_Command(.5));
        // leftButton5.whileFalse(new Claw1_Command(0));



    }


}
