package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import com.stzteam.features.marsprocessor.CreateCommand;
import com.stzteam.features.marsprocessor.RequestFactory;
import com.stzteam.mars.diagnostics.ActionStatus;
import com.stzteam.mars.diagnostics.ModuleColorCode;
import com.stzteam.mars.diagnostics.StatusColorCode.Severity;
import com.stzteam.mars.requests.Request;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Interfaces.TurretIO;
import frc.robot.Interfaces.TurretIO.TurretInputs;

@RequestFactory
public interface TurretRequests extends Request<TurretInputs, TurretIO> {

    public static final ModuleColorCode IDLE = ModuleColorCode.solid("Idle", Severity.OK, Color.kGreen, "Brazo en reposo");
    public static final ModuleColorCode MANUAL_MOVEMENT = ModuleColorCode.solid("Manual control", Severity.WARNING, Color.kTeal, "Operador moviendo");
    public static final ModuleColorCode TRACKING = ModuleColorCode.solid("Reaching target", Severity.WARNING, Color.kLightBlue, "Llendo al setpoint");
    public static final ModuleColorCode LOCKED = ModuleColorCode.solid("Reaching target", Severity.OK, Color.kGreen, "En el setpoint");

    @CreateCommand (name = "stop")
    public static class Idle implements TurretRequests{
        @Override
        public ActionStatus apply(TurretInputs data, TurretIO actor) {
            actor.stop();
            return ActionStatus.of(IDLE);
        }
    }

    @CreateCommand (name = "manual_control")
    public static class Manual_control implements TurretRequests{
        private DoubleSupplier stick;

        public Manual_control joystick (DoubleSupplier stick){
            this.stick = stick;
            return this;
        }

        @Override
        public ActionStatus apply(TurretInputs data, TurretIO actor){
            
            if (data.angle.getDegrees()>-90 && data.angle.getDegrees()<90){
                actor.setDutyCycle(stick.getAsDouble()*0.5);
            } else {
                actor.stop();
            }
            return ActionStatus.of(MANUAL_MOVEMENT);
        }
    }

        @CreateCommand (name = "set_position")
    public static class SetPosition implements TurretRequests{
        private Rotation2d angle;
        private double toleranceDegrees;

        public SetPosition withTargetAngle (Rotation2d angle){
            this.angle = angle;
            return this;
        }
        
        public SetPosition withTolerance ( double degrees){
            this.toleranceDegrees = degrees;
            return this;
        }

        @Override
        public ActionStatus apply (TurretInputs data, TurretIO actor){
            actor.setPosition(angle);

            boolean locked = 
                MathUtil.isNear(angle.getDegrees(), data.angle.getDegrees(), toleranceDegrees);

            if (locked) {
            return ActionStatus.of( LOCKED, "Turret locked");
            } else{
            return ActionStatus.of(TRACKING, "Tracking objective");
            }
        }

    }


}
