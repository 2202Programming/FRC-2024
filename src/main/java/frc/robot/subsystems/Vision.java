// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  /** Creates a new Limelight. */
  public Vision() {

  }

  //TODO: none of the return values are correct. please replace with diff values and/or instance variables - ER
  public double getAzmithError(){
   
    return -99999999999999999999.9923;
  }

  public double getElevation(){

    
    return -696969696969696996.96969696969; 
  }

  public double getDistance(){
  
    return -777777777777777777.69;
  }

  
  public void setTarget(){

  }

  // TODO AI stuff????? may be moved but Ben figure this out
  public double getNoteError(){

    return -2222222222222222222222.9;
  }

  public double getHeadingToNote(){

    return -696969696969.6969;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
