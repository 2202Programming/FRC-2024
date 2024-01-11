// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ExampleSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

    //Instantiations 
  final CANSparkMax mtr = new CANSparkMax(20, CANSparkMax.MotorType.kBrushless);

  private double currentMotorSpeed;

  public ExampleSubsystem() {
    motor_config(mtr, false);
    currentMotorSpeed = mtr.get();
  }


  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  public void setMotorSpeed(double motorSpeed){
    mtr.set(motorSpeed);
    currentMotorSpeed = motorSpeed;
  }

  public double getMotorSpeed(){
    return currentMotorSpeed;
  }

  void motor_config(CANSparkMax mtr, boolean inverted) {
    mtr.clearFaults();
    mtr.restoreFactoryDefaults();
    mtr.setInverted(inverted);
 }

}
