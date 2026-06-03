package frc.robot.Bindings;

import com.stzteam.mars.models.containers.Binding;
import com.stzteam.mars.operator.ControllerOI;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.TurretRequestsFactory;
import frc.robot.Mechanisms.Turret;


@SuppressWarnings("unused")
public class OperatorBindings implements Binding{

    private final ControllerOI operator;
    private final Turret turret;
    private final Rotation2d ninentyDegrees = new Rotation2d(90);
    private final Rotation2d zero = new Rotation2d();

    public OperatorBindings( ControllerOI operator, Turret turret){
        this.operator = operator;
        this.turret = turret;
    }

    public static OperatorBindings Create(ControllerOI operator, Turret turret){
        return new OperatorBindings(operator, turret);
    }

    @Override
    public void bind(){

        double DEADBAND = 0.1;
        var buttons = operator.getActionButtons();
        var bumpers = operator.getBumpers();
        var operatorSystem = operator.getSystemTriggers();
        var leftStick = operator.getLeftStick();
        var rightStick = operator.getRightStick();
        var triggers = operator.getAnalogTriggers();
        var pov = operator.getDPadTriggers();

        Trigger rightStickXTrigger = new Trigger(() -> Math.abs(rightStick.x().getAsDouble()) > DEADBAND);
        Trigger rightStickYTrigger = new Trigger(() -> Math.abs(rightStick.y().getAsDouble()) > DEADBAND);
        Trigger leftStickXTrigger = new Trigger(() -> Math.abs(leftStick.x().getAsDouble()) > DEADBAND);
        Trigger leftStickYTrigger = new Trigger(() -> Math.abs(leftStick.y().getAsDouble()) > DEADBAND);

        leftStickXTrigger.and(pov.right()).whileTrue(turret.setControl(
            ()-> TurretRequestsFactory.manual_control().joystick(leftStick.x())));
        buttons.top().onTrue(turret.setControl(
            ()-> TurretRequestsFactory.setPosition().withTargetAngle(new Rotation2d(80)).withTolerance(5)));
        buttons.bottom().onTrue(turret.setControl(
            ()-> TurretRequestsFactory.setPosition().withTargetAngle(new Rotation2d(-80)).withTolerance(5)));
    }
    
}
