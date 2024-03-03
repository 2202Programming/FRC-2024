// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Sensors.Sensors_Subsystem;


public class AccelWatcher extends Command {
    final Sensors_Subsystem sensors;
    // final SwerveDrivetrain drivetrain;
    final double d_time = Constants.DT;
    final double targ_accel;
    boolean exceeded_target;
    double accel; // [m/s^2]
    boolean use_absolute;
    boolean use_neg_case;

    Command cmd = null;  // command to run when accel event happens

    /*
     * 
     * Watches the X axis acceleration and finishes when the acceleration is greater
     * than the targ_accel.
     * 
     * Useful to tell when drivetrain hits a wall
     * 
     * Mr.L - changed to use sensors.XAccel instead of calculating a velocity
     * 
     */
    public AccelWatcher(double targ_accel, boolean use_absolute) {
        this.sensors = RobotContainer.getSubsystem(Sensors_Subsystem.class);
        // this.drivetrain = RobotContainer.getSubsystem(SwerveDrivetrain.class);
        this.targ_accel = targ_accel;
        this.use_absolute = use_absolute;
        if (use_absolute) {
            targ_accel = Math.abs(targ_accel);
        }
    }

    public AccelWatcher(double targ_accel) {
        this(targ_accel, true);
    }
    public AccelWatcher(Command cmd, double targ_accel, boolean use_absolute) {
        this(targ_accel, use_absolute);
        this.cmd = cmd;
    }
    @Override
    public void initialize() {
        exceeded_target = false;
    }

    @Override
    public void execute() {
        accel = sensors.getXAccel();
        accel = (use_absolute) ? Math.abs(accel) : accel;
        if (targ_accel < 0.0) {
            exceeded_target = (accel < targ_accel);
        } else {
            exceeded_target = (accel > targ_accel);
        }
    }
    @Override
    public void end(boolean interrupted) {
        if (cmd != null && !interrupted) {
            cmd.schedule();
            System.out.println("Accel limit hit, scheduling: " + cmd.getName() );
        }
    }

    @Override
    public boolean isFinished() {
        return exceeded_target;
    }
}
