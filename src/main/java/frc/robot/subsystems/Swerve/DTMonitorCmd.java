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
    NetworkTable table = getTable();
    var tname = getTableName();
    // use smartdashboard for complex object
    //SmartDashboard.putData(tname + "/drive PIDF", DriveTrain.drivePIDF);

    currentX = table.getEntry("current x");
    currentY = table.getEntry("current y");
    currentHeading = table.getEntry("current heading");

    velocityFL = table.getEntry("drive FL velocity");
    velocityFR = table.getEntry("drive FR velocity");
    velocityBL = table.getEntry("drive BL velocity");
    velocityBR = table.getEntry("drive BR velocity");
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

    // robot coordinates - speeds
    var speeds = sdt.getChassisSpeeds();

    // todo speeds.vxMetersPerSecond
    // speeds.vyMetersPerSecond
    // speeds.omegaRadiansPerSecond

  }

}
