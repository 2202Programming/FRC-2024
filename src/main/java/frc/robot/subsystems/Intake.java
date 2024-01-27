// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import com.revrobotics.CANSparkMax;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN;
import frc.robot.Constants.PCM1;
import frc.robot.Constants.DigitalIO;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
    static final Value DEPLOY = Value.kReverse;
    static final Value RETRACT = Value.kForward;
    final CANSparkMax intake_mtr = new CANSparkMax(CAN.INTAKE_MTR, CANSparkMax.MotorType.kBrushless);
    final DigitalInput lightgate = new DigitalInput(DigitalIO.IntakeLightGate);
    final DoubleSolenoid left_solenoid = new DoubleSolenoid(CAN.PCM1,PneumaticsModuleType.REVPH, PCM1.LT_INTAKE_UP_SOLENOID_PCM, PCM1.LT_INTAKE_DOWN_SOLENOID_PCM);
    final DoubleSolenoid right_solenoid = new DoubleSolenoid(CAN.PCM1, PneumaticsModuleType.REVPH, PCM1.RT_INTAKE_UP_SOLENOID_PCM, PCM1.RT_INTAKE_DOWN_SOLENOID_PCM);
  public Intake() {
    motor_config(intake_mtr);
  }
  void motor_config(CANSparkMax motor){
    motor.clearFaults();
    motor.restoreFactoryDefaults();
  }
  public void setMotorSpeed(double speed){
    intake_mtr.set(speed);
  }
  public boolean hasNote(){
    return lightgate.get();
  }
  public double getMotorSpeed(){
    return intake_mtr.get();
  }
  public void intakeOff(){
    intake_mtr.set(0);
  }
  public void deploy(){
    left_solenoid.set(DEPLOY);
    right_solenoid.set(DEPLOY);
  }
  public void retract(){
    left_solenoid.set(RETRACT);
    right_solenoid.set(RETRACT);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
