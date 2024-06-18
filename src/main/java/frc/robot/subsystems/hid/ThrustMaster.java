package frc.robot.subsystems.hid;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;

public class ThrustMaster  extends CommandJoystick{


  public ThrustMaster(final int port) {
    super(port);
  }
  
  public enum Buttons{
     UpTop(2),  
    LeftTop(3), RightTop(4), LeftOne(5), LeftTwo(6), LeftThree(7), LeftFour(8), 
    LeftFive(9), LeftSix(10), RightOne(11), 
    RightTwo(12),
    RightThree(13),
    RightFour(14), RightFive(15), RightSix(18), /* , PinkyTopTriggerRed(19),
    TriggerGn(16), PinkyTriggerGn(17), TopTriggerGn(20), PinkyTopTriggerGn(21), */
    Mode(30);     //this will also switch trigger modes   - number one

    public int value;
    private Buttons(final int val) {
      value = val;
    }
  }

  public boolean getUpTop(){
    return getHID().getRawButton(Buttons.UpTop.value);
  }
  
}
