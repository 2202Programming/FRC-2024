// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Constants.DriveTrain;
import frc.robot.subsystems.Swerve.Config.CANConfig;
import frc.robot.subsystems.Swerve.Config.ChassisConfig;
import frc.robot.subsystems.Swerve.Config.ChassisInversionSpecs;
import frc.robot.subsystems.Swerve.Config.WheelOffsets;

public class RobotSpecs {
    public enum RobotNames {
        CompetitionBot2024("CompetitionBot2024"), 
        SwerveBot("SwerveBot"),
        CompetitionBot2023("DoofBot2023"),
        ChadBot("ChadBot"),
        UnknownBot("UnknownBot"),
        BotOnBoard("BotOnBoard");

        String name;

        private RobotNames(String name) {
            this.name = name;
        }

        public String toString() {
            return name;
        }
    }
    
    /*Configs*/
    public RobotNames myRobotName;
    private WheelOffsets myWheelOffsets;
    private ChassisConfig myChassisConfig;
    private SubsystemConfig mySubsystemConfig;
    private ChassisInversionSpecs myChassisInversionSpecs;
    private CANConfig myCANConfig;

    public RobotSpecs() {
        this(System.getenv("serialnum"));
    }

    public RobotSpecs(String serialNo) {
        myRobotName = getRobotName(serialNo);

        // if we are simulated, use the competionBot so we have everything
        if (RobotBase.isSimulation()) {
            myRobotName = RobotNames.CompetitionBot2024;
        }
        // setup to handle any swerve both we have
        switch (myRobotName) {
            case SwerveBot:
                myWheelOffsets = DriveTrain.swerveBotOffsets;
                myChassisConfig = DriveTrain.swerveBotChassisConfig;
                mySubsystemConfig = DriveTrain.swerveBotSubsystemConfig;
                myChassisInversionSpecs = DriveTrain.swerveBotChassisInversionSpecs;
                myCANConfig = DriveTrain.swerveBotCANConfig;
                break;
                
            case CompetitionBot2024:
                myWheelOffsets = DriveTrain.comp2024BotOffsets;
                myChassisConfig = DriveTrain.comp2024BotChassisConfig;
                mySubsystemConfig = DriveTrain.comp2024BotSubsystemConfig;
                myChassisInversionSpecs = DriveTrain.comp2024BotInversionSpecs;
                myCANConfig = DriveTrain.comp2024BotCANConfig;
                break;

            case CompetitionBot2023:  //doofBot
                myWheelOffsets = DriveTrain.doofBotOffsets;
                myChassisConfig = DriveTrain.doofBotChassisConfig;
                mySubsystemConfig = DriveTrain.doofBotSubsystemConfig;
                myChassisInversionSpecs = DriveTrain.doofBotChassisInversionSpecs;
                myCANConfig = DriveTrain.doofBotCANConfig;
                break;

            case ChadBot:
                myWheelOffsets = DriveTrain.chadBotOffsets;
                myChassisConfig = DriveTrain.chadBotChassisConfig;
                mySubsystemConfig = DriveTrain.chadBotSubsystemConfig;
                myChassisInversionSpecs = DriveTrain.chadBotChassisInversionSpecs;
                myCANConfig = DriveTrain.chadBotCANConfig;
                break;

            default:
            case UnknownBot:
            case BotOnBoard:
                myWheelOffsets = DriveTrain.swerveBotOffsets;
                myChassisConfig = DriveTrain.swerveBotChassisConfig;
                mySubsystemConfig = DriveTrain.botOnBoardConfig;
                myChassisInversionSpecs = DriveTrain.swerveBotChassisInversionSpecs;
                myCANConfig = DriveTrain.swerveBotCANConfig;
                System.out.println("***Non-driving robot,don't expect too much***");
                break;
        }
        System.out.println("***I am " + myRobotName + " ***");        
    }

    public String getRobotNameString() {
        return this.myRobotName.name;
    }

    public WheelOffsets getWheelOffset() {
        return myWheelOffsets;
    }

    public ChassisConfig getChassisConfig() {
        return myChassisConfig;
    }

    public SubsystemConfig getSubsystemConfig() {
        return mySubsystemConfig;
    }

    public ChassisInversionSpecs getChassisInversionSpecs(){
        return myChassisInversionSpecs;
    }

    public CANConfig getCANConfig(){
        return myCANConfig;
    }

    // takes the roborio serial # and returns the robot name
    public RobotNames getRobotName(String serialNo) {
        RobotNames tempRobotName;

        if (serialNo == null)
            return RobotNames.UnknownBot;

        if (serialNo.compareTo("031b7511") == 0)
            tempRobotName = RobotNames.SwerveBot;
        else if (serialNo.compareTo("03238151") == 0)
            tempRobotName = RobotNames.ChadBot;
        else if (serialNo.compareTo("0312db1a") == 0)
            tempRobotName = RobotNames.BotOnBoard;
        else if (serialNo.compareTo("031b7523") == 0)
            tempRobotName = RobotNames.CompetitionBot2023;
        else if (serialNo.compareTo("032381BF") == 0) 
            tempRobotName = RobotNames.CompetitionBot2024;
        else
            tempRobotName = RobotNames.UnknownBot;

        System.out.println("***RoboRio SERIAL NUM: " + serialNo);
        System.out.println("***Robot identified as: " + tempRobotName);
        return tempRobotName;
    }
}
