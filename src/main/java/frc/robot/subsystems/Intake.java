package frc.robot.subsystems;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN;
import frc.robot.Constants.DigitalIO;
import frc.robot.commands.Lights;
import frc.robot.util.NeoServo;
import frc.robot.util.PIDFController;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
    public double intake_speed = 0.0;
    public double r_speed = 0.0;
    public double l_speed = 0.0;
    static final Value DEPLOY = Value.kReverse;
    static final Value RETRACT = Value.kForward;
    final NeoServo angle_mtr;
    final CANSparkMax intake_mtr = new CANSparkMax(CAN.INTAKE_MTR, CANSparkMax.MotorType.kBrushless);
    PIDController positionPID = new PIDController(0.0, 0.0, 0.0); //outer (pos)
    PIDFController hwVelPID = new PIDFController(0.0, 0.0, 0.0, 0.0); //inner (hw/vel)
    final DigitalInput lightgate = new DigitalInput(DigitalIO.IntakeLightGate);
  public Intake() {
    angle_mtr = new NeoServo(CAN.ANGLE_MTR, positionPID, hwVelPID, false); //should be invert false
    motor_config(intake_mtr);
    angle_config(angle_mtr);
  }
  void angle_config(NeoServo angle) {
    // rest of Neo and servo PID stuff
    angle.setConversionFactor(0.0) // add below later when we actually need values - NR 1/31/24
            // .setSmartCurrentLimit(STALL_CURRENT, FREE_CURRENT)
            // .setVelocityHW_PID(maxVel, maxAccel)
            // .setTolerance(posTol, velTol)
            // .setMaxVelocity(maxVel)
            .burnFlash();
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
  public void setAngleSetpoint(double position){
    angle_mtr.setSetpoint(position);
  }
  public double getAngleSetpoint(){
    return angle_mtr.getSetpoint();
  }
  public double getAnglePosition(){
    return angle_mtr.getPosition();
  }
  public double getAngleSpeed(){
    return angle_mtr.getVelocity();
  }
  public void setClamp(double min_ext, double max_ext){ //limit switch values?
    angle_mtr.setClamp(min_ext, max_ext);
  }
    //Returns the state of the Intake Arm
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(lightgate.get()){
      new Lights(BlinkyLights.GREEN);
    }
    new Lights(BlinkyLights.RED);
  }
}
