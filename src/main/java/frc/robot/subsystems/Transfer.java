// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN;
import frc.robot.Constants.DigitalIO;
import frc.robot.Constants.Transfer_Constants;
import frc.robot.commands.utility.WatcherCmd;
import frc.robot.subsystems.Intake.IntakeWatcherCmd;
import frc.robot.util.PIDFController;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

public class Transfer extends SubsystemBase {

  DigitalInput lightgate = new DigitalInput(DigitalIO.TRANSFER_LIGHT_GATE);
  CANSparkMax transferMtr;
  final SparkPIDController transferMtrPid;
  final RelativeEncoder transferMtrEncoder;
  final PIDFController transferPID = new PIDFController(0.0, 0.0, 0.0, 1.0);

  /** Creates a new Transfer. */
  public Transfer() {
    final double radius = 1.27 * 2 * Math.PI; // 1.27 radius in cm
    final double gear_ratio = 35.0;
    double conversionFactor = radius / gear_ratio;
    conversionFactor = 1.0;
    transferMtr = new CANSparkMax(CAN.TRANSFER_MOTOR, CANSparkMax.MotorType.kBrushless);
    transferMtr.clearFaults();
    transferMtr.restoreFactoryDefaults();
    transferMtr.setInverted(true);

    transferMtrPid = transferMtr.getPIDController();
    transferMtrEncoder = transferMtr.getEncoder();
    transferMtrEncoder.setPositionConversionFactor(conversionFactor);
    transferMtrEncoder.setVelocityConversionFactor(conversionFactor / 60.0); // min to sec
    transferPID.copyTo(transferMtrPid, 0);
    transferMtr.burnFlash();
  }

  // TODO: find out methods/behaviors, pneumatics, etc.

  public boolean hasNote() {
    return !lightgate.get();
  }

  public void transferMtrOn() {
    // transferMtrPid.setReference(Transfer_Constants.TRANSFER_MOTOR_ON, ControlType.kVelocity, 0);
    transferMtr.set(Transfer_Constants.TRANSFER_MOTOR_ON);
  }

  // Motor speed will likely need to be chan
  public void transferMtrOff() {
    // transferMtrPid.setReference(Transfer_Constants.TRANSFER_MOTOR_OFF, ControlType.kVelocity, 0);
     transferMtr.set(Transfer_Constants.TRANSFER_MOTOR_OFF);
  }

  public void transferMtrReverse() {
    transferMtrPid.setReference(Transfer_Constants.TRANSFER_MOTOR_REVERSE, ControlType.kVelocity, 0);
    // transferMtr.set(Transfer_Constants.TRANSFER_MOTOR_REVERSE);
  }

  public double getTransferVelocity() {
    return transferMtrEncoder.getVelocity();
  }

  public Command getWatcher() {
    return new TransferWatcherCmd();
  }

  // This motor speed will also probably need to be changed too, but make sure it
  // is still a negative number
  // This method would be used to spit the note back out if it gets jammed, but
  // might not be necessary
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  class TransferWatcherCmd extends WatcherCmd {
    // NetworkTableEntry nt_lightgate;
    NetworkTableEntry nt_lightgate;
    NetworkTableEntry nt_transferVel;

    @Override
    public String getTableName() {
      return Transfer.this.getName();
    }

    public void ntcreate() {
      NetworkTable table = getTable();
      // nt_lightgate = table.getEntry("lightgate");
      nt_lightgate = table.getEntry("lightgate");
      nt_transferVel = table.getEntry("transferVel");

      // default value for mutables
      // example nt_maxArbFF.setDouble(maxArbFF);
    }

    public void ntupdate() {
      // nt_lightgate.setBoolean();
      nt_lightgate.setBoolean(hasNote());
      nt_transferVel.setDouble(getTransferVelocity());

      // get mutable values
      // example maxArbFF = nt_maxArbFF.getDouble(maxArbFF);

    }
  } // watcher command
}
