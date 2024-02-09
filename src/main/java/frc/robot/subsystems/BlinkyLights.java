// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.CANdleControlFrame;
import com.ctre.phoenix.led.CANdleStatusFrame;
import com.ctre.phoenix.led.StrobeAnimation;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CAN;

public class BlinkyLights {
    static final int TO = 50; // [ms] timeout for comms

    // candle frame update rate, larger saves on CAN BUS
    static final int FrameStatusTime = 200; // [ms]

    // Some common colors
    static public Color8Bit BLACK = new Color8Bit(0, 0, 0);
    static public Color8Bit WHITE = new Color8Bit(255, 255, 255);
    static public Color8Bit RED = new Color8Bit(255, 0, 0);
    static public Color8Bit GREEN = new Color8Bit(0, 255, 0);
    static public Color8Bit BLUE = new Color8Bit(0, 0, 255);
    static public Color8Bit ORANGE = new Color8Bit(255, 145, 0);

    // State vars
    CANdle candle_l;
    CANdle candle_r;
    double brightness;
    Color8Bit currentColor;
    boolean amIReal;

    // make sure we have one user to service any calls
    static BlinkyLightUser defaultUser = new BlinkyLightUser() {
        @Override
        public Color8Bit colorProvider() {
            return ORANGE;
        }
        // other overides could go here for the default
    };

    /** Creates a new BlinkyLights. */
    public BlinkyLights() {
        BlinkyLightController.setDefaultUser(defaultUser);
        candle_l = new CANdle(CAN.CANDLE1);
        candle_r = new CANdle(CAN.CANDLE2);

        // see if there is a bus voltage, if so we must be real
        amIReal = (candle_l.get5VRailVoltage() > 1.0);
        amIReal = true;

        if (amIReal) {
            System.out.println("***I have blinkylights, I must be one of the cool robots.");
            config(candle_l);
            config(candle_r);

            BlinkyLightController.controlledLights = this;
            setBrightness(1.0);

        } else {
            // no lights
            candle_l = null;
            candle_r = null;
            System.out.println("***I have no blinkylights :( ... setting up poser methods");
        }
    }

    void config(CANdle cdl) {
        final var StatusFrame = CANdleStatusFrame.CANdleStatusFrame_Status_1_General;

        var cfg = new CANdleConfiguration();
        cdl.clearStickyFaults(TO);
        cfg.enableOptimizations = true;
        cdl.configAllSettings(cfg, TO);

        // lower CAN bus usage for CANdle
        var period = cdl.getStatusFramePeriod(StatusFrame);
        if (period < FrameStatusTime)
            cdl.setStatusFramePeriod(StatusFrame, FrameStatusTime, TO);

        cdl.setControlFramePeriod(CANdleControlFrame.CANdle_Control_1_General, FrameStatusTime);
        cdl.setControlFramePeriod(CANdleControlFrame.CANdle_Control_2_ModulatedVBatOut, FrameStatusTime);
    }

    void setColor(Color8Bit color) {
        if (amIReal) {
            candle_l.setLEDs(color.red, color.green, color.blue);
            candle_r.setLEDs(color.red, color.green, color.blue);
        }
        currentColor = color;
    }

    void setBlinking(boolean blink) {
        if (blink)
            setBlinking(currentColor);
        else
            stopBlinking();
    }

    void setBlinking(Color8Bit color) {
        if (amIReal) {
            Animation animation = new StrobeAnimation(color.red, color.green, color.blue, 0, 0.5, 8);
            candle_l.animate(animation, 0);
            candle_r.animate(animation, 0);
        }
    }

    void stopBlinking() {
        if (amIReal) {
            candle_l.clearAnimation(0);
            candle_r.clearAnimation(0);
        }
    }

    /*
     * Brightness on a scale from 0-1, with 1 being max brightness
     */
    void setBrightness(double brightness) {
        if (amIReal) {
            candle_l.configBrightnessScalar(brightness);
            candle_r.configBrightnessScalar(brightness);
        }
    }

    public void setAllianceColors() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        // no alliance indicated
        if (alliance.isEmpty()) {
            setColor(BLACK);
            return;
        }

        // System.out.println("***** Robot Alliance: " +
        // DriverStation.getAlliance().name());
        switch (alliance.get()) {
            case Blue:
                setColor(BLUE);
                break;
            case Red:
                setColor(RED);
                break;
            default:
                setColor(new Color8Bit(0, 255, 0));
                break;
        }
    }

    static boolean differentColors(Color8Bit c1, Color8Bit c2) {
        return (c1.red != c2.red) ||
                (c1.green != c2.green) ||
                (c1.blue != c2.blue);
    }


    /**
     * BlinkyLightUser a command for controllering of the lights
     */
    // commands and sub-systems
    public static class BlinkyLightUser extends Command {
        public void onRobotInit() {
        };

        public void onAutomousInit() {
        };

        public void onTeleopInit() {
        };

        public void onTestInit() {
        };

        // used in commands, Override to your preferences
        public Color8Bit colorProvider() {
            return WHITE;
        };

        // used in commands, Override to your preferences
        public boolean requestBlink() {
            return false;
        }

        public void enableLights() {
            // user is a Command, setup a parallel cmd to get the color info
            var watchCmd = new BlinkyLightController.UserWatcherCommand(this);
            watchCmd.schedule();
        }
    }

    //TODO make a real subsystem?
    // static because there can be only one controller for now
    // tracks who is using the lights and runs the allianceColor during
    // disabled periodic.
    public static class BlinkyLightController {
        protected static BlinkyLights controlledLights = null;
        private static BlinkyLightUser defaultUser = null;
        private static BlinkyLightUser currentUser = defaultUser;

        public static void setDefaultUser(BlinkyLightUser default_user) {
            defaultUser = default_user;
            setCurrentUser(default_user);
        }

        public static void setCurrentUser(BlinkyLightUser user) {
            currentUser = user;
        };

        public static void release() {
            currentUser = defaultUser;
            if (controlledLights != null) {
                controlledLights.setColor(ORANGE);
            }
        };

        /* Static methods to intercept robot state changes */
        public static void onRobotInit() {
            currentUser.onRobotInit();
        };

        public static void onDisabledPeriodic() {
            // always do our alliance colors
            if (controlledLights != null)
                controlledLights.setAllianceColors();
        }

        public static void onAutomousdInit() {
            currentUser.onAutomousInit();
        };

        public static void onTeleopInit() {
            currentUser.onTeleopInit();
        };

        public static void onTestInit() {
            currentUser.onTestInit();
        };

        public static void onTestPeriodic() {
            currentUser.onTestInit();
        };

        static class UserWatcherCommand extends Command {
            Color8Bit currentColor;
            boolean blinkState;
            final BlinkyLightUser myUser; // a command that is using the lights
            final Command watcherCmd; // a watcher to get values to the CANdles periodically

            UserWatcherCommand(BlinkyLightUser user) {
                myUser = user;
                watcherCmd = this;
            }

            UserWatcherCommand() {
                myUser = null;
                watcherCmd = this;
            }

            /*
             * Pulls initial values from the BLUser
             */
            @Override
            public void initialize() {
                setCurrentUser(myUser);
                if (controlledLights == null)
                    return;

                currentColor = myUser.colorProvider();
                controlledLights.setColor(currentColor);
                blinkState = myUser.requestBlink();
                controlledLights.setBlinking(blinkState);
            }

            /*
             * Reads providers and send to CANDles.
             * 
             * This can could be setup to happen less frequently
             */
            @Override
            public void execute() {
                Color8Bit newColor = myUser.colorProvider();
                if (controlledLights == null)
                    return;

                // avoid CAN bus traffic if color isn't changing
                if (!currentColor.equals(newColor)) {
                    currentColor = newColor;
                    controlledLights.setColor(currentColor);
                }
                if (myUser.requestBlink() != blinkState) {
                    blinkState = myUser.requestBlink();
                    controlledLights.setBlinking(blinkState);
                }
            }

            // watcher should always be able to
            @Override
            public boolean runsWhenDisabled() {
                return myUser.runsWhenDisabled();
            }

            @Override
            public void end(boolean interrupted) {
                release();
            }

            @Override
            public boolean isFinished() {
                // run until my parent command is done
                return !myUser.isScheduled();
            }
        }

    }
}// static class BlinkyLightController
