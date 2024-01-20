package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class Blinkin {
    private Spark m_blinkin;

    /** Creates a new LEDs. */
    public Blinkin() {
      m_blinkin = new Spark(LEDConstants.kArmBlinkinPort);
    }
    public void set(double val) {
        m_blinkin.set(val);
    }
    public void setFireEffect(){
        m_blinkin.set(LEDConstants.kFireEffect);
    }
}
