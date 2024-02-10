// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN;
import frc.robot.Constants.DigitalIO;
import frc.robot.commands.utility.WatcherCmd;
import frc.robot.util.NeoServo;
import frc.robot.util.PIDFController;

public class NoseRoller extends SubsystemBase {
  /** Creates a new NoseRoller. */
  final NeoServo noseAngleMotor;
  public CANSparkMax noseFireMotor = new CANSparkMax(CAN.NOSE_MOTOR_FIRE, CANSparkMax.MotorType.kBrushless);
  final PIDFController hwVelPID = new PIDFController(1.0, 0.0, 0.0, 0.0); // inner (hw/vel)
  final PIDController posPID = new PIDController(1.0, 0.0, 0.0);
  final DigitalInput lightgate = new DigitalInput(DigitalIO.RollerLightGate);
  public double initialPosition; // Should be our "0" where the nose roller's rollers are in the poition to take
  // the piece
  public double noseMaxSpeed; // RPM

  public NoseRoller() {
    noseAngleMotor = new NeoServo(CAN.NOSE_MOTOR_ANGLE, posPID, hwVelPID, false);

  }

  public void setRollerSpeed(double speed) {
    noseFireMotor.set(speed);
  }

  public boolean rollerHasNote() {
    return lightgate.get();
  }

  public double getNosePosition() {
    return noseAngleMotor.getPosition();
  }

  public void setNosePosition(double pos) {
    noseAngleMotor.setSetpoint(pos);
  }

  public void setNoseVelocity(double vel) {
    noseAngleMotor.setVelocityCmd(vel);
  }

  public boolean isGateBlocked() { // if we even have a gate?
    return false;
  }
  public boolean isAtAngle(){
    return noseAngleMotor.atSetpoint();
  }

  // access to get internal watcher command
  public Command getWatcher() {
    return new NoseRollerWatcherCmd();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /*
   * Watcher for this subsystem - schedule this command if you need to
   * see valuse on the Dashboard.
   */
  class NoseRollerWatcherCmd extends WatcherCmd {
    NetworkTableEntry nt_lightgate;

    @Override
    public String getTableName() {
      return NoseRoller.this.getName();
    }

    public void ntcreate() {
      NetworkTable table = getTable();
      nt_lightgate = table.getEntry("lightgate");

      // default value for mutables
      // example nt_maxArbFF.setDouble(maxArbFF);
    }

    public void ntupdate() {
      nt_lightgate.setBoolean(isGateBlocked());

      // get mutable values
      // example maxArbFF = nt_maxArbFF.getDouble(maxArbFF);

    }
  } // watcher command

} // noseroller
