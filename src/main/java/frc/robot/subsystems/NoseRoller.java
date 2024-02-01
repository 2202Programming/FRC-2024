// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;


import frc.robot.Constants.CAN;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NoseRoller extends SubsystemBase {
  /** Creates a new NoseRoller. */
    public CANSparkMax noseAngleMotor = new CANSparkMax(CAN.NOSE_MOTOR_ANGLE, CANSparkMax.MotorType.kBrushless);
    public CANSparkMax noseFireMotor = new CANSparkMax(CAN.NOSE_MOTOR_FIRE, CANSparkMax.MotorType.kBrushless);
    public double initialPosition; //Should be our "0" where the nose roller's rollers are in the poition to take the piece
    public double noseMaxSpeed; //RPM
  
   
  public NoseRoller() {

  }

  public void setMotorsToStart() {

  }

  public double getNosePosition() {
    return 0.0;
  }

  public void setNosePosition(double pos) {
  
  }

  public void sneeze() {
    noseFireMotor.set(1.0); //this should proboably not fire at max speed. Proboably 
  }

  public void sniffle() {
    noseFireMotor.set(0.0);
  }


  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
