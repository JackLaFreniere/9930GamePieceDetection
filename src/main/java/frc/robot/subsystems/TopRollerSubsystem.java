package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.SparkMaxWrapper;

public class TopRollerSubsystem extends SubsystemBase{

    private final CANSparkMax topRoller;

    public static final double ROLLER_INTAKE_SPEED = 0.4;

    public TopRollerSubsystem(int topRollerID) {
        topRoller = new SparkMaxWrapper(topRollerID, MotorType.kBrushless);

        topRoller.restoreFactoryDefaults();
        topRoller.setIdleMode(IdleMode.kBrake);
        
        topRoller.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);
        topRoller.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);

        topRoller.setInverted(true);
    }

    /**<h3>getRollerSpeed</h3>
     * Sets the top roller speed
     * @return setRollerSpeed
     */
    public void setRollerSpeed(double speed) {
        topRoller.set(speed);
        Logger.getInstance().recordOutput(this.getClass().getSimpleName() + "/TopRollerspeed", speed);
    }
}
