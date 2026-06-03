package frc.robot.Mechanisms;

import java.util.function.Supplier;

import com.stzteam.forgemini.io.NetworkIO;
import com.stzteam.mars.models.SubsystemBuilder;
import com.stzteam.mars.models.Telemetry;
import com.stzteam.mars.models.singlemodule.ModularSubsystem;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Commands.TurretRequests;
import frc.robot.Commands.TurretRequestsCommands;
import frc.robot.Interfaces.TurretIO;
import frc.robot.Interfaces.TurretIO.TurretInputs;
import frc.robot.configuration.KeyManager;

public class Turret extends ModularSubsystem<TurretInputs, TurretIO> implements TurretRequestsCommands {

    public Turret (TurretIO io){
        super(
            SubsystemBuilder.<TurretInputs,TurretIO> setup()
                .key(KeyManager.TURRET_KEY)
                .hardware(io, new TurretInputs())
                .request(new TurretRequests.Idle())
                .telemetry(new TurretTelemetry())
        );
        setDefaultCommand(runRequest(()-> new TurretRequests.Idle()));
    }

    @Override
    public TurretInputs getState(){
        return inputs;
    }

    @Override
    public Command setControl(Supplier<TurretRequests> request){
        return runRequest(request);
    }

    @Override
    public void absolutePeriodic(TurretInputs data){}

    @Override 
    public void simulationPeriodic(){}

    public static class TurretTelemetry extends Telemetry<TurretInputs>{

        @Override
        public void telemeterize(TurretInputs data) {
            NetworkIO.set(KeyManager.TURRET_KEY, "Position", data.angle);
            NetworkIO.set(KeyManager.TURRET_KEY, "Target angle", data.targetAngle);
            NetworkIO.set(KeyManager.TURRET_KEY, "Applied voltage", data.voltage);
            NetworkIO.set(KeyManager.TURRET_KEY, "Target angle", data.speed);
        }
    }

    public boolean isAtTarget(double toleranceDegrees, Turret turret){
        boolean isAtTarget = 
        MathUtil.isNear(turret.getState().targetAngle.getDegrees(), turret.getState().angle.getDegrees(), toleranceDegrees);
        return isAtTarget;
    }

    
} 

