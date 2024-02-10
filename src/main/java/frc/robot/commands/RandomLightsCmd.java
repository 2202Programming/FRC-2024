// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.subsystems.BlinkyLights;
import frc.robot.subsystems.BlinkyLights.BlinkyLightUser;

/*
 * An example of how to use the blinky lights in your command.
 * 
 * initialize()  - call enablelights()
 * 
 * @overide the colorProvider function with something meaningful for your command
 * colorProvider() - returns the Color8Bit you want
 * 
 * That's it!
 */
public class RandomLightsCmd extends BlinkyLightUser {
  /** Creates a new Lights Command */
  final int FRAMES = 10; // change color every N frames
  private Color8Bit myColor;
  private int count = 0;

  public RandomLightsCmd(Color8Bit Color) {
    myColor = Color;
  }

  public RandomLightsCmd() {
    this(BlinkyLights.ORANGE);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    count = 0;
    enableLights();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (++count < FRAMES)
      return;

    // pick a new random color
    count = 0;
    myColor = new Color8Bit(
        (int) (Math.random() * 255),
        (int) (Math.random() * 255),
        (int) (Math.random() * 255));
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  //required by blinkylightuser if you want the lights to change
  @Override
  public Color8Bit colorProvider() {
    return myColor;
  }

}
