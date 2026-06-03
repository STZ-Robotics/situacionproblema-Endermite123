package frc.robot.configuration;

import com.stzteam.mars.builder.Environment;
import com.stzteam.mars.builder.Injector;
import com.stzteam.mars.builder.Environment.RunMode;
import com.stzteam.mars.operator.ControllerOI;
import com.stzteam.mars.operator.PS5OI;
import com.stzteam.mars.operator.XboxOI;

import frc.robot.Interfaces.TurretIO;
import frc.robot.Interfaces.TurretIOFallback;
import frc.robot.Mechanisms.Turret;
import frc.robot.Motors.TurretIOSpark;
import frc.robot.Simulation.TurretSim;

public class Manifest {

    public static final boolean HAS_TURRET = true;

    public enum ControllerType{PS5,XBOX}
    public static final int driver_port =0;
    public static final int operator_port =1;

    public static final RunMode CURRENT_MODE = RunMode.REAL;

    static{Environment.setMode(CURRENT_MODE);}

    public static final ControllerType driver = ControllerType.XBOX;
    public static final ControllerType operator = ControllerType.XBOX;

    public static class ControlsBuilder{
        public static ControllerOI buildDriver(){
            return driver == ControllerType.PS5
            ? new PS5OI(driver_port)
            : new XboxOI(driver_port);
        }
        public static ControllerOI buildOperator(){
            return operator == ControllerType.PS5
            ? new PS5OI(operator_port)
            : new XboxOI(operator_port);
        }
    }

    public static Turret buildTurret(){
        TurretIO io = Injector.createIO(HAS_TURRET, TurretIOFallback::new, TurretIOSpark::new, TurretSim::new);
        return new Turret(io);
    } 


}
