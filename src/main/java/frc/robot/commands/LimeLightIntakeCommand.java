package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.utilities.LimeLightUtility;

public class LimeLightIntakeCommand extends CommandBase {
    private SwerveDrive m_SwerveDrive;
    private LimeLightUtility m_LimeLight;
    private String m_LimeLightName;

    private double m_MaxStrafe = 0.1;
    
    private double m_throttle;
    private double m_strafe;

    private static double kDt = 0.0;
    private final TrapezoidProfile.Constraints m_constraints =
        new TrapezoidProfile.Constraints(1.0, 1.0);
    private TrapezoidProfile.State m_goal = new TrapezoidProfile.State(Units.feetToMeters(3.0), 0);
    private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State(0,0);

    private TrapezoidProfile profile = new TrapezoidProfile(m_constraints, m_goal, m_setpoint);

    private PIDController pid = new PIDController(0.0, 0.0, 0.0);

    public LimeLightIntakeCommand(SwerveDrive swerveDrive, String limeLightName) {
        m_SwerveDrive = swerveDrive;
        m_LimeLightName = limeLightName;
        addRequirements(m_SwerveDrive);
    }

    @Override
    public void execute() {
        m_strafe = MathUtil.clamp(-m_LimeLight.get_tx(m_LimeLightName), -m_MaxStrafe, m_MaxStrafe);
        
        m_setpoint = profile.calculate(kDt);
        m_throttle = m_setpoint.velocity;
        kDt = kDt + 0.02;

        m_SwerveDrive.drive(m_throttle, m_strafe, 0.0, false, true);
    }

    @Override
    public boolean isFinished() {
        return profile.isFinished(kDt);
    }

    @Override
    public void end(boolean interrupted) {
        kDt = 0.0;
    }
}
