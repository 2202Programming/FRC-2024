// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Swerve;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveTrain;
import frc.robot.commands.utility.WatcherCmd;

public class DTMonitorCmd extends WatcherCmd {

  /** Creates a new DTMonitorCmd. */

  // Use addRequirements() here to declare subsystem dependencies.
  NetworkTableEntry currentX;
  NetworkTableEntry currentY;
  NetworkTableEntry currentHeading;

  NetworkTableEntry velocityFL;
  NetworkTableEntry velocityFR;
  NetworkTableEntry velocityBL;
  NetworkTableEntry velocityBR;

  NetworkTableEntry FL_pos;
  NetworkTableEntry FR_pos;
  NetworkTableEntry BL_pos;
  NetworkTableEntry BR_pos;

  NetworkTableEntry radiansPerSecond;
  NetworkTableEntry xMetersPerSec;
  NetworkTableEntry yMetersPerSec;


  // accessors - if this gets annoying move inside
  final SwerveDrivetrain sdt;
  final SwerveModuleMK3 modules[] = new SwerveModuleMK3[4];

  public DTMonitorCmd() {
    sdt = RobotContainer.getSubsystem(SwerveDrivetrain.class);
  }

  @Override
  public String getTableName() {
    return SwerveDrivetrain.class.getSimpleName();
  }

  @Override
  public void ntcreate() {
    NetworkTable MonitorTable = getTable();
    var tname = getTableName();
    // use smartdashboard for complex object
    SmartDashboard.putData(tname + "/drive PIDF", DriveTrain.drivePIDF);

    currentX = MonitorTable.getEntry("current x");
    currentY = MonitorTable.getEntry("current y");
    currentHeading = MonitorTable.getEntry("current heading");

    velocityFL = MonitorTable.getEntry("drive FL velocity");
    velocityFR = MonitorTable.getEntry("drive FR velocity");
    velocityBL = MonitorTable.getEntry("drive BL velocity");
    velocityBR = MonitorTable.getEntry("drive BR velocity");

    FL_pos = MonitorTable.getEntry("front left position");
    FR_pos = MonitorTable.getEntry("front right position");
    BL_pos = MonitorTable.getEntry("back left position");
    BR_pos = MonitorTable.getEntry("back right position");

    radiansPerSecond = MonitorTable.getEntry("chassis radians sec");
    xMetersPerSec = MonitorTable.getEntry("velocity x meters sec");
    yMetersPerSec = MonitorTable.getEntry("velocity y meters sec");

  }

  @Override
  public void ntupdate() {
    // DriveTrain.drivePIDF should be handled via SmartDash buildable
    // read values from swerve drivetrain as needed using accesors
    var m_pose = sdt.getPose();

    currentX.setDouble(m_pose.getX());
    currentY.setDouble(m_pose.getY());
    currentHeading.setDouble(m_pose.getRotation().getDegrees());

    // read swerve module values (FL FR BL BR) with best api we have
    for (int i = 0; i < modules.length; i++) {
      modules[i] = sdt.getMK3(i);
    }
    // unpack modules
    velocityFL.setDouble(modules[0].getVelocity());
    velocityFR.setDouble(modules[1].getVelocity());
    velocityBL.setDouble(modules[2].getVelocity());
    velocityBR.setDouble(modules[3].getVelocity());

    FL_pos.setDouble(modules[0].getPosition());
    FR_pos.setDouble(modules[1].getPosition());
    BL_pos.setDouble(modules[2].getPosition());
    BR_pos.setDouble(modules[3].getPosition());

    // robot coordinates - speeds
    var speeds = sdt.getChassisSpeeds();
    radiansPerSecond.setDouble(speeds.omegaRadiansPerSecond * 57.3);
    xMetersPerSec.setDouble(speeds.vxMetersPerSecond);
    yMetersPerSec.setDouble(speeds.vyMetersPerSecond);

    // todo speeds.vxMetersPerSecond 
    // speeds.vyMetersPerSecond
    // speeds.omegaRadiansPerSecond

  }

}
