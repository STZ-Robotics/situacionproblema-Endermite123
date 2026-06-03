package frc.tests;

import com.stzteam.mars.test.MARSTest;
import com.stzteam.mars.test.TestRoutine;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Commands.TurretRequestsFactory;
import frc.robot.Mechanisms.Turret;

@MARSTest (name = "TurretTest")
public class TurretTest extends TestRoutine {
    private final Turret t;

    public TurretTest (Turret turret){
        this.t = turret;
    }
    
    @Override 
    public Command getRoutineCommand(){
        return Commands.sequence(
            run(TurretRequestsFactory.setPosition().withTargetAngle(new Rotation2d(45)).withTolerance(5),t),
            waitFor(()->t.isAtTarget(5,t ), 1),
            assertLessThan(calculateError(Units.degreesToRadians(45), t.getState().angle.getRadians()), 2, "High turret error on target 1"),
            delay(1),
            run(TurretRequestsFactory.setPosition().withTargetAngle(new Rotation2d(-45)).withTolerance(5),t),
            waitFor(()->t.isAtTarget(5,t ), 1),
            assertLessThan(calculateError(Units.degreesToRadians(-45), t.getState().angle.getRadians()), 2, "High turret error on target 1"),
            delay(1),
            run(TurretRequestsFactory.setPosition().withTargetAngle(new Rotation2d(0)).withTolerance(5),t),
            waitFor(()->t.isAtTarget(5,t ), 1),
            assertLessThan(calculateError(Units.degreesToRadians(0), t.getState().angle.getRadians()), 2, "High turret error on target 1"),
            run (TurretRequestsFactory.idle(),t)
        );
    }
}