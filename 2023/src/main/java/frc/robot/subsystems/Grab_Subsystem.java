package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kForward;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kOff;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kReverse;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class Grab_Subsystem extends SubsystemBase

{
    DoubleSolenoid clawSolenoidA = null;

    

    public Grab_Subsystem()
    {
        clawSolenoidA = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
        clawSolenoidA.set(kForward);
        
    }

    // @Override
    // public void initDefaultCommand() 
    // {
    //     setDefaultCommand(new Grab_Command());
    // }
    public void grabToggle()
    {
        clawSolenoidA.toggle();
    }

    public void grabClose() // lines 35-40 not used
    {
        clawSolenoidA.set(kReverse);
    }
    

}
