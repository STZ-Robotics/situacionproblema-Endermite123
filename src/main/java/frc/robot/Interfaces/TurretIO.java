package frc.robot.Interfaces;

import com.stzteam.features.marsprocessor.Fallback;
import com.stzteam.mars.models.singlemodule.Data;
import com.stzteam.mars.models.singlemodule.IO;

import edu.wpi.first.math.geometry.Rotation2d;

@Fallback
public interface TurretIO extends IO<TurretIO.TurretInputs> {
    public static class TurretInputs extends Data<TurretInputs> {
        public Rotation2d targetAngle = new Rotation2d();
        public Rotation2d angle = new Rotation2d();
        public double rps = 0;
        public double voltage = 0;
        public double current = 0;

        @Override
        public TurretInputs snapshot() {
        TurretInputs clone = new TurretInputs();
        
        clone.timestamp = this.timestamp;
        clone.key = this.key;
        clone.angle = this.angle;
        clone.targetAngle = this.targetAngle;
        clone.rps = this.rps;
        clone.voltage = this.voltage;
        clone.current = this.current;
        return clone;
}
    }

    public void setVoltage(double voltage);
    public void setDutyCycle(double speeed);
    public void setPosition(Rotation2d angle);
    public void setPositionWithFF(Rotation2d angle, double feedforward);
    public void resetEncoder();
    public void stop();

}
