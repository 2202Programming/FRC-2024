package frc.robot.auto;

import java.util.List;


import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.hid.CommandSwitchboardController;
import frc.robot.subsystems.hid.HID_Xbox_Subsystem;
import frc.robot.subsystems.hid.SwitchboardController.SBButton;

public class autoCommand {
    double maxVelocity = 3.00; //TODO made up number
    double maxAcceleration = 3.00; //TODO made up number
    HID_Xbox_Subsystem driverControl = RobotContainer.RC().dc;

    public autoCommand(){

    }

    public void initialize(){
        List<PathPlannerPath> pathGroup;

        if (driverControl.readSideboard(null)){ //TODO button
            System.out.println("Running red auto path 1");
            pathGroup = PathPlannerAuto.getPathGroupFromAutoFile("redSussex");
        }

        else if (driverControl.readSideboard(null)) { //TODO button
            System.out.println("Running blue auto path 2");
            pathGroup = PathPlannerAuto.getPathGroupFromAutoFile("blueSussex");
        }

        else {
            System.out.println("No auto");
            return;
        }

        SequentialCommandGroup tempCommand = new SequentialCommandGroup(RobotContainer.RC().autoBuilder.fullAuto(pathGroup.get(0)));

        tempCommand.schedule();
    }

    public void execute() {

    }
}
