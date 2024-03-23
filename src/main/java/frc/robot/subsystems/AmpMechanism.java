// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CAN;
import frc.robot.commands.utility.WatcherCmd;
import frc.robot.util.NeoServo;
import frc.robot.util.PIDFController;

public class AmpMechanism extends SubsystemBase {
  public final static double StartPosition = 0.0; //[cm] or [deg]?? decide
  public final static double ExtendPosition = 28.0; //[cm]

  final double GearRatio = 1.0; // TODO: find
  final double maxVel = 100.0; // placeholder. cm/s?
  final double maxAccel = 10.0; // placevholder cm/s^2
  double posTol = 0.25; // [cm]
  double velTol = 0.50; // [cm/s]
  final int STALL_CURRENT = 5; // placeholder // units?
  final int FREE_CURRENT = 15; // placeholder // units?
  /** Creates a new AmpMechanism. */
  final PIDController posPID = new PIDController(1.0, 0.0, 0.0); //find
  final PIDFController hwVelPID = new PIDFController(1.0, 0.0, 0.0, 0.0); //find
  final NeoServo angleServo = new NeoServo(Constants.CAN.AMP_MECHANISM, posPID, hwVelPID, false);
  public AmpMechanism() {
    hwVelPID.copyTo(angleServo.getController().getPIDController(), 0);
    angleServo.setConversionFactor(GearRatio) // in cm
        .setSmartCurrentLimit(STALL_CURRENT, FREE_CURRENT)
        .setVelocityHW_PID(maxVel, maxAccel)
        .setTolerance(posTol, velTol)
        .setMaxVelocity(maxVel)
        .burnFlash();
    angleServo.setPosition(StartPosition);

  }
  public void setVelocity(double vel){
    angleServo.setVelocityCmd(vel);
  }
  public double getCmdVelocity(){
    return angleServo.getVelocityCmd();
  }
  public double getVelocity(){
    return angleServo.getVelocity();
  }
  public void setSetpoint(double pos){
    angleServo.setSetpoint(pos);
  }
  public double getSetpoint(){
    return angleServo.getSetpoint();
  }
  public double getPosition(){
    return angleServo.getPosition();
  }
  public void setPosition(double pos){
    angleServo.setPosition(pos);
  }
  public boolean atSetpoint(){
    return angleServo.atSetpoint();
  }
  public Command getWatcher(){
    return new AmpMechanismWatcherCmd();
  }

  @Override
  public void periodic() {
    angleServo.periodic();
    // This method will be called once per scheduler run
  }

  class AmpMechanismWatcherCmd extends WatcherCmd {
    NetworkTableEntry nt_desiredVel;
    NetworkTableEntry nt_desiredPos;
    NetworkTableEntry nt_currentVel;
    NetworkTableEntry nt_currentPos;
    NetworkTableEntry nt_atSetpoint;

    @Override
    public String getTableName() {
      return AmpMechanism.this.getName();
    }

    public void ntcreate() {
      NetworkTable table = getTable();
      nt_desiredVel = table.getEntry("desiredSpeed");
      nt_desiredPos = table.getEntry("desiredPos");
      nt_currentVel = table.getEntry("currentSpeed");
      nt_currentPos = table.getEntry("currentPos");
      nt_atSetpoint = table.getEntry("atSetpoint");

    }

    public void ntupdate() {
      nt_desiredVel.setDouble(getCmdVelocity());
      nt_desiredPos.setDouble(getSetpoint());
      nt_currentVel.setDouble(getVelocity());
      nt_currentPos.setDouble(getPosition());
      nt_atSetpoint.setBoolean(atSetpoint());
    }
  } // watcher command
}

