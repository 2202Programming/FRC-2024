package frc.robot.util;

import java.lang.reflect.InvocationTargetException;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Subsystem;

/*
* SubsystemConfig - handle different flavors of robot  sub-system building.
* Objects will be constructed in order they are added. This is critical for
* systems like Drivetrain which requires Sensors and maybe Vision to be 
* available.
*
*  See Configs.java for correct usage for different platforms we have.
*/
public class SubsystemConfig {
    /* SubsystemDefinition keeps class and subsystem for use in a has map */
    static class SubsystemDefinition<T extends Object> {
        Class<T> m_Class;
        Object m_obj = null;
        Supplier<Object> m_factory = null;
        String m_alias = null;

        // container for Subsystem Class and instance object
        SubsystemDefinition(Class<T> clz) {
            m_Class = clz;
            m_obj = null;
        }

        SubsystemDefinition(Class<T> clz, String alias) {
            m_Class = clz;
            m_obj = null;
            m_alias = alias;
        }

        @SuppressWarnings("unchecked")
        SubsystemDefinition(T obj) {
            this.m_Class = (Class<T>) obj.getClass();
            this.m_obj = obj;
        }

        SubsystemDefinition(Class<T> clz, Supplier<Object> factory) {
            this.m_Class = clz;
            this.m_obj = null;
            this.m_factory = factory;
        }

        /*
         * construct the subsystem using the default constructor.
         */
        void construct() {
            // already created
            if (m_obj != null)
                return;
            
            // alias fixed in constructAll()
            if (m_alias != null) 
                return;

            // have a Factory supplied lambda
            if (m_factory != null) {
                m_obj = m_factory.get();
                return;
            }

            // use a no-args constructor for anything without a factory lambda
            try {
                // Use the no-args constructor to create instance
                m_obj = m_Class.getDeclaredConstructor().newInstance();

            } catch (NoSuchMethodException e) {
                System.out.println("*** Problem creating " + m_Class.getSimpleName()
                        + " a no-arg constructor is required.  Object not created. ***");
                e.printStackTrace();

            } catch (InstantiationException | IllegalAccessException | IllegalArgumentException | SecurityException
                    | InvocationTargetException e) {
                // handle other cases that shouldn't happen for Subsystems
                System.out.println("***Problem creating Object "
                        + m_Class.getName() + "... Subsystem not created.***");
                e.printStackTrace();
            }
        }
    }

    // map all the robot subsystem by a name
    LinkedHashMap<String, SubsystemDefinition<?>> m_robot_parts = new LinkedHashMap<>();

    /*
     * An alias gives an existing Object another name.
     * 
     * This is useful for cases where a command can work with different types of Subsystems.
     * For example, different drivetrains can support pathing or rotation resets, but
     * should not be tied to a specif class.  DRIVETRAINs may fit this use case.
     */

    /*
     * creates an alias name for the given subsystem
     */
    public <T> SubsystemConfig addAlias(Class<T> clz, String alias) {
        String name = clz.getSimpleName();
        var ssd = new SubsystemDefinition<T>(clz, alias);
        put(name, ssd);
        return this;
    }

    public <T> SubsystemConfig add(Class<T> clz) {
        String name = clz.getSimpleName();
        var ssd = new SubsystemDefinition<T>(clz);
        put(name, ssd);
        return this;
    }

    public <T> SubsystemConfig add(Class<T> clz, String name) {        
        put(name, new SubsystemDefinition<T>(clz));
        return this;
    }

    public <T> SubsystemConfig add(T instance, String name) {        
        put(name, new SubsystemDefinition<T>(instance));
        return this;
    }

    // Facory adds must have a Type and a name
    public <T> SubsystemConfig add(Class<T> clz, String name, Supplier<Object> factory) {
        put(name, new SubsystemDefinition<T>(clz, factory));
        return this;
    }

    // Only subsystems will be returned
    public Subsystem getSubsystem(String name) {
        // Not sure how to get rid of this warning
        var ssd = get(name);
        return (ssd.m_obj instanceof Subsystem) ? (Subsystem) ssd.m_obj : null;
    }

    // Only subsystems will be returned
    @SuppressWarnings("unchecked")
    public <T extends Subsystem> T getSubsystem(Class<? extends Subsystem> clz) {
        // Not sure how to get rid of this warning
        String name = clz.getSimpleName();
        var ssd = get(name);
        return (ssd.m_obj instanceof Subsystem) ? (T) ssd.m_obj : null;
    }

    // Any robot object will be returned, including Subsystems. 
    // Class name must be unique to the configuration.
    @SuppressWarnings("unchecked")
    public <T> T getObject(String name) {
        var ssd = get(name);
        return (T) ssd.m_obj;
    }
    // Any robot object will be returned, including Subsystems. 
    // Class name must be unique to the configuration.
    @SuppressWarnings("unchecked")
    public <T> T getObjectOrNull(String name) {
       var ssd = m_robot_parts.get(name); 
        return (T) ssd.m_obj;
    
    }
    // Only subsystems will be returned
    @SuppressWarnings("unchecked")
    public <T extends Subsystem> T getSubsystemOrNull(Class<? extends Subsystem> clz) {
        // Not sure how to get rid of this warning
        String name = clz.getSimpleName();
        var ssd = m_robot_parts.get(name); 
        return (ssd.m_obj instanceof Subsystem) ? (T) ssd.m_obj : null;
    }
    public boolean has(String name) {
        return m_robot_parts.containsKey(name);
    }

    // for when it's not a Subsystem, and a factory wasn't used
    public boolean has(Class<?> clz) {       
        String n = clz.getSimpleName();
        return has(n);
    }

    // for when it must be a subsystem
    public boolean hasSubsystem(Class<?> clz) {
        if (!clz.isNestmateOf(Subsystem.class))
            return false; // clz not Subsystem
        String n = clz.getSimpleName();
        return has(n);
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
        for (Map.Entry<String, SubsystemDefinition<?>> entry : m_robot_parts.entrySet()) {
            System.out.println("Constructing " + entry.getKey() + " as instance of " +
                    entry.getValue().m_Class.getSimpleName());
            entry.getValue().construct();

            //handle alias which must be defined After the instance is created.
            var ssd = entry.getValue();
            if (ssd.m_alias != null) {
                ssd.m_obj = get(ssd.m_Class.getSimpleName()).m_obj;
            }
        }
    }

    void put(String name, SubsystemDefinition<?> ssd) {
        // see if name exist, if so that is a problem as names must be unique
        if (m_robot_parts.containsKey(name)) {
            System.out.println("*********************************************\n"
                    + "SubsystemConfig contains DUPLICATE NAME "
                    + name + "of Class " + ssd.m_Class.getCanonicalName()
                    + " duplicate will not be created."
                    + "*********************************************\n");
        }
        m_robot_parts.put(name, ssd);
    }

     SubsystemDefinition<?> get(String name) {
        var ssd = m_robot_parts.get(name); 
        if (ssd != null) return ssd;

        // doesn't exist, fail hard and fast
        System.out.println("SubsystemConfig: your object " + name + " does not exist.\n"
        + "Check your Configs.java file\n" 
        + "Throwing NPE to keep you safe.");
        throw new NullPointerException();
    }


}
