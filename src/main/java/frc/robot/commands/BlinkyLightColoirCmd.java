// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.RobotContainer;
import frc.robot.subsystems.BlinkyLights;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.ShooterServo;
import frc.robot.subsystems.Transfer;
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
public class BlinkyLightColoirCmd extends BlinkyLightUser {
  /** Creates a new Lights Command */
  private Color8Bit myColor;
  ShooterServo shooter = RobotContainer.getSubsystem(ShooterServo.class);
  Transfer transfer = RobotContainer.getSubsystem(Transfer.class);
  Intake intake = RobotContainer.getSubsystem(Intake.class);

  public BlinkyLightColoirCmd(Color8Bit Color) {
    myColor = Color;
  }

  public BlinkyLightColoirCmd() {
    this(BlinkyLights.RED);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    enableLights();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(DriverStation.getMatchTime() < 15.0){
      myColor = new Color8Bit(Color.kOrange);
    }
    else if(intake.senseNote()){
      myColor = new Color8Bit(Color.kYellow);
    }
   else if(transfer.hasNote()){
    myColor = new Color8Bit(Color.kGreen);
   }
   else{
    myColor = new Color8Bit(Color.kRed);
   }
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
