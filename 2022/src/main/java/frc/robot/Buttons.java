package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.commands.Claw1_Command;
import frc.robot.commands.Claw2_Command;
import frc.robot.commands.Claw3_Command;
import frc.robot.commands.Lift_Command;
import frc.robot.commands.WheelSpin_Command;




public class Buttons 
{
    public Joystick leftJoy = new Joystick(0);
    public Joystick rightJoy = new Joystick(1);



    public Joystick getLeftJoystick() { return leftJoy; } 
    public Joystick getRightJoystick() { return rightJoy; }

    Button leftButton4 = new JoystickButton (leftJoy, 4); //claw 1 open
    Button leftButton5 = new JoystickButton(leftJoy, 5); //claw 1 close

    Button leftButton6 = new JoystickButton(leftJoy, 6); //claw 2 open
    Button leftButton7 = new JoystickButton(leftJoy, 7); //claw 2 close

    Button leftButton10 = new JoystickButton(leftJoy, 10); //claw 3 open
    Button leftButton11 = new JoystickButton(leftJoy, 11); //claw 3 close

    Button rightButton2 = new JoystickButton(rightJoy, 2); //lift down (LS)
    Button rightButton3 = new JoystickButton(rightJoy, 3); //lift up (LS)

    Button rightButton4 = new JoystickButton(rightJoy, 4); //Wheel CCW
    Button rightButton5 = new JoystickButton(rightJoy, 5); //Wheel CW

    Button rightButton10 = new JoystickButton(rightJoy, 10);//Fine Tune Wheel CCW
    Button rightButton11 = new JoystickButton(rightJoy, 11);//Fine Tune Wheel CW

    Button rightButton6 = new JoystickButton(rightJoy, 6); //Claw 1 toggle
    Button rightButton7 = new JoystickButton(rightJoy, 7); //Claw 2 toggle
    Button rightButton8 = new JoystickButton(rightJoy, 8); //Claw 3 toggle



    public Buttons() 
    {
                    //Claw 1//
    leftButton4.whenPressed(new Claw1_Command(.4));
    leftButton4.whenReleased(new Claw1_Command(0));
    leftButton5.whenPressed(new Claw1_Command(-.4));
    leftButton5.whenReleased(new Claw1_Command(0));

                    //Claw 2//
    leftButton6.whenPressed(new Claw2_Command(-.4));
    leftButton6.whenReleased(new Claw2_Command(0));
    leftButton7.whenPressed(new Claw2_Command(.4));
    leftButton7.whenReleased(new Claw2_Command(0));

                    //Claw 3//
    leftButton10.whenPressed(new Claw3_Command(.4));
    leftButton10.whenReleased(new Claw3_Command(0));
    leftButton11.whenPressed(new Claw3_Command(-.4));
    leftButton11.whenReleased(new Claw3_Command(0));

                    //Lift System//
    rightButton2.whenPressed(new Lift_Command(-.5));//Down
    rightButton2.whenReleased(new Lift_Command(0));
    rightButton3.whenPressed(new Lift_Command(.8));//Up
    rightButton3.whenReleased(new Lift_Command(0));

                    //Wheel Rotation Fine Tune//
    rightButton10.whenPressed(new WheelSpin_Command(-.5));
    rightButton10.whenReleased(new WheelSpin_Command(0));
    rightButton11.whenPressed(new WheelSpin_Command(.5));
    rightButton11.whenReleased(new WheelSpin_Command(0));

                 //Wheel Rotation//
     rightButton4.whenPressed(new WheelSpin_Command(-1));
     rightButton4.whenReleased(new WheelSpin_Command(0));
     rightButton5.whenPressed(new WheelSpin_Command(1));
     rightButton5.whenReleased(new WheelSpin_Command(0));
            

                    //Claw Toggle//

    // rightButton6.toggleWhenPressed(new Claw1_Command(-.2));
    // rightButton7.toggleWhenPressed(new Claw2_Command(.2));
    // rightButton8.toggleWhenPressed(new Claw3_Command(.2));




    


    }


}
 
 

 