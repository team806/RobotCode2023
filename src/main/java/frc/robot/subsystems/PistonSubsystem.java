package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PistonSubsystem extends SubsystemBase{
    private final DoubleSolenoid m_grabberPiston;

    public PistonSubsystem(int solenoidID1, int solenoidID2) {
        m_grabberPiston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, solenoidID1, solenoidID2);
    }

    public void toggle() {
        m_grabberPiston.toggle();
    }

    public boolean isOpen() {
        if(m_grabberPiston.get() == DoubleSolenoid.Value.kReverse) {
            return true;
        }
        else {
            return false;
        }
    }

    public void close() {
        m_grabberPiston.set(DoubleSolenoid.Value.kReverse);
    }

    public void open() {
        m_grabberPiston.set(DoubleSolenoid.Value.kForward);
    }
    public void stop() {
        //m_grabberPiston.set(DoubleSolenoid.Value.kOff);
    }

}
