package frc.robot.util;

import java.lang.reflect.InvocationTargetException;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Subsystem;

/*
* SubsystemConfig - handle different flavors of robot 
* sub-system building.
*  
*/

public class SubsystemConfig {

    /* SubsystemDefinition keeps class and subsystem for use in a has map */
    static class SubsystemDefinition {
        Class<?> m_Class;
        Object m_obj = null;
        Supplier<Object> m_factory = null;

        // container for Subsystem Class and instance object
        SubsystemDefinition(Class<? extends Subsystem> clz) {
            m_Class = clz;
            m_obj = null;
        }

        SubsystemDefinition(Object obj) {
            this.m_Class = obj.getClass();
            this.m_obj = obj;
        }

        SubsystemDefinition(Class<?> clz, Supplier<Object> factory) {
            this.m_Class = clz;
            this.m_obj = null;
            this.m_factory = factory;
        }

        /*
         * construct the subsystem using the default constructor.
         */
        void construct() {
            if (m_obj != null)
                return;
            if (m_factory == null &&  m_Class.isNestmateOf(Subsystem.class)) {

                System.out.println("Subysystem construct: I don't know how to make a " + m_Class.getName() +
                        "\n - no factory and not Subsystem class.\n" +
                        "Continuing with robot construction... good luck!");
                return; // skip non subsystems
            }

            // do Factory supplied first
            if (m_factory != null) {
                m_obj = m_factory.get();
                return;
            }

            // require a no-args constructor
            try {
                // assume no-args constructor, create subsystem instance
                m_obj = m_Class.getDeclaredConstructor().newInstance();

            } catch (NoSuchMethodException e) {
                System.out.println(
                        "***Subsystem " + m_Class.getName()
                                + "needs a no-arg constructor... Subsystem not created.***");
                e.printStackTrace();

            } catch (InstantiationException | IllegalAccessException | IllegalArgumentException
                    | InvocationTargetException e) {
                System.out.println(
                        "***Problem creating Subsystem " + m_Class.getName() + "... Subsystem not created.***");
                e.printStackTrace();
            }
        }
    }

    // map all the robot subsystem by a name
    LinkedHashMap<String, SubsystemDefinition> m_robot_parts = new LinkedHashMap<>();

    // only one instance of a class, use UPPER case name of class for instance
    public SubsystemConfig add(Class<? extends Subsystem> clz) {
        String name = clz.getSimpleName().toUpperCase();
        var ssd = new SubsystemDefinition(clz);
        m_robot_parts.put(name, ssd);
        return this;
    }

    public <T> SubsystemConfig add(Class<? extends Subsystem> clz, String instance_name) {
        String name = instance_name.toUpperCase();
        m_robot_parts.put(name, new SubsystemDefinition(clz));
        return this;
    }

    public <T> SubsystemConfig add(Subsystem instance, String instance_name) {
        String name = instance_name.toUpperCase();
        m_robot_parts.put(name, new SubsystemDefinition(instance));
        return this;
    }

    public <T> SubsystemConfig add(Class<?> clz, String name, Supplier<Object> factory) {
        m_robot_parts.put(name.toUpperCase(), new SubsystemDefinition(clz, factory));
        return this;
    }

    // Only subsystems will be returned
    public Subsystem getSubsystem(String instance_name) {
        // Not sure how to get rid of this warning
        var ssd = m_robot_parts.get(instance_name);
        return (ssd.m_obj instanceof Subsystem) ? (Subsystem) ssd.m_obj : null;
    }

    // Any robot object will be returned, including Subsystems
    public Object getObject(String instance_name) {
        // Not sure how to get rid of this warning
        var ssd = m_robot_parts.get(instance_name);
        return ssd.m_obj;
    }

    public boolean has(String instance_name) {
        return m_robot_parts.containsKey(instance_name);
    }

    public boolean hasSubsystem(Class<? extends Subsystem> clz) {
        String n = clz.getSimpleName().toUpperCase();
        return m_robot_parts.containsKey(n);
    }

    /*
     * constructAll() - call all the subsystem constructors if they haven't been
     * initialized
     * 
     * This should be called in RobotContainer for its only its system config after
     * all the robot specs
     * are setup.
     */
    public void constructAll() {
        for (Map.Entry<String, SubsystemDefinition> entry : m_robot_parts.entrySet()) {
            System.out.println("Constructing " + entry.getKey() + " as instance of " + 
                entry.getValue().m_Class.getSimpleName());
            entry.getValue().construct();
        }
    }

}
