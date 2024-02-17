// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Swerve;

import edu.wpi.first.math.geometry.Pose2d;
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

  NetworkTableEntry est_pose_od_x;
  NetworkTableEntry est_pose_od_y;
  NetworkTableEntry est_pose_od_h;
  NetworkTableEntry est_pose_integ_x;
  NetworkTableEntry est_pose_integ_y;
  NetworkTableEntry est_pose_integ_h;


  // accessors - if this gets annoying move inside
  final SwerveDrivetrain sdt;
  final SwerveModuleMK3 modules[] = new SwerveModuleMK3[4];
  Pose2d nt_pose;
  Pose2d nt_pose_integ; // incorperates vision

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

    est_pose_od_x = MonitorTable.getEntry("est_od_x");
    est_pose_od_y = MonitorTable.getEntry("est_od_y");
    est_pose_od_h = MonitorTable.getEntry("est_od_h");
    est_pose_integ_x = MonitorTable.getEntry("est_int_x");
    est_pose_integ_y = MonitorTable.getEntry("est_int_y");
    est_pose_integ_h = MonitorTable.getEntry("est_int_h");
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

    est_pose_od_x.setDouble(nt_pose.getX());
    est_pose_od_y.setDouble(nt_pose.getY());
    est_pose_od_h.setDouble(nt_pose.getRotation().getDegrees());

    est_pose_integ_x.setDouble(nt_pose_integ.getX());
    est_pose_integ_y.setDouble(nt_pose_integ.getY());
    est_pose_integ_h.setDouble(nt_pose_integ.getRotation().getDegrees());

  }
}
