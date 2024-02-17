// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN;
import frc.robot.Constants.DigitalIO;
import frc.robot.commands.utility.WatcherCmd;
import frc.robot.util.PIDFController;

public class Transfer extends SubsystemBase {

  //constants for geometry of transfer
  final static double radius = 1.27 * 2.0 * Math.PI; // 1.27 radius in cm
  final static double gearRatio = 1.0 / 35.0; // 35 motor turns -> 1 roller shaft turn
  final static double conversionFactor = radius * gearRatio;  // [cm/rotations]

  static final double MIN_SPEED=-1.0, MAX_SPEED=1.0; //example looks like Pct Pwr

  // calc Kff for vel control from measured (RPS / %pwr)
  final static double  Kff =  (1.0 - .25) / (186.0 - 46.0);
  final PIDFController transferPID = new PIDFController(0.0, 0.0, 0.0, Kff);

  DigitalInput lightgate = new DigitalInput(DigitalIO.TRANSFER_LIGHT_GATE);
  CANSparkMax transferMtr;
  final SparkPIDController transferMtrPid;
  final RelativeEncoder transferMtrEncoder;

  // state vars
  boolean has_note = false;
  boolean prev_sense_note = false;

  /** Creates a new Transfer. */
  public Transfer() {
    transferMtr = new CANSparkMax(CAN.TRANSFER_MOTOR, CANSparkMax.MotorType.kBrushless);
    transferMtr.clearFaults();
    transferMtr.restoreFactoryDefaults();
    transferMtr.setInverted(true);
    transferMtrPid = transferMtr.getPIDController();
    transferMtrEncoder = transferMtr.getEncoder();
    transferMtrEncoder.setPositionConversionFactor(conversionFactor);
    transferMtrEncoder.setVelocityConversionFactor(conversionFactor / 60.0); // min to sec
    transferPID.copyTo(transferMtrPid, 0);
    transferMtrPid.setOutputRange(MIN_SPEED, MAX_SPEED, 0);
    transferMtr.burnFlash();
  }

  /*
   * true when note is blocking light gate
   */
  boolean senseNote() {
    return !lightgate.get();
  }

  /*
   * true - note passed light gate and then cleared past gate
   * commands should stop motor on hasNote() == true.
   */
  public boolean hasNote() {
    return has_note;
  }

  /*
   * sets if we have a note or not for powerup or initization in commands
   */
  public void setHasNote(boolean note_state) {
      has_note = note_state;
      prev_sense_note = false;
  }

  /*
   * speed [cm/s]
   */
  public void setSpeed(double speed) {
     transferMtrPid.setReference(speed, ControlType.kVelocity, 0);
    // transferMtr.set(Transfer_Constants.TRANSFER_MOTOR_ON);
  }

  /*
  // Motor speed will likely need to be chan
  public void transferMtrOff() {
    // transferMtrPid.setReference(Transfer_Constants.TRANSFER_MOTOR_OFF,
    // ControlType.kVelocity, 0);
   // transferMtr.set(Transfer_Constants.TRANSFER_MOTOR_OFF);
  }

 /*
  public void transferMtrReverse() {
    transferMtrPid.setReference(Transfer_Constants.TRANSFER_MOTOR_REVERSE, ControlType.kVelocity, 0);
    // transferMtr.set(Transfer_Constants.TRANSFER_MOTOR_REVERSE);
  }
*/
  public double getTransferVelocity() {
    return transferMtrEncoder.getVelocity();
  }

  public Command getWatcher() {
    return new TransferWatcherCmd();
  }

  /*
   * watch gate during periodic so the hasNote() is accurate
   */
  @Override
  public void periodic() {
    // watch gate for high to low change, we have the note where we want it
    if (senseNote()){
      prev_sense_note = true;
    }
    else if (prev_sense_note) {
      has_note = true;   // saw it then didn't, so we move past we have it
    }

  }

  class TransferWatcherCmd extends WatcherCmd {
    // NetworkTableEntry nt_lightgate;
    NetworkTableEntry nt_lightgate;
    NetworkTableEntry nt_transferVel;
    NetworkTableEntry nt_have_note;

    @Override
    public String getTableName() {
      return Transfer.this.getName();
    }

    public void ntcreate() {
      NetworkTable table = getTable();
      nt_lightgate = table.getEntry("senseNote");
      nt_transferVel = table.getEntry("transferVel");
      nt_have_note = table.getEntry("haveNote");

      // default value for mutables
      // example nt_maxArbFF.setDouble(maxArbFF);
    }

    public void ntupdate() {
      // nt_lightgate.setBoolean();
      nt_lightgate.setBoolean(senseNote());
      nt_transferVel.setDouble(getTransferVelocity());
      nt_have_note.setBoolean(hasNote());

      // get mutable values
      // example maxArbFF = nt_maxArbFF.getDouble(maxArbFF);

    }
  } // watcher command
}
