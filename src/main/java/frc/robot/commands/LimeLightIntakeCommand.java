package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.utilities.LimeLightUtility;

/**
 * 
 * <h3>LimeLightIntakeCommand</h3>
 * 
 * Uses game piece detection and a Trapezoidal Profile to smoothly and accurately intake a cube during autonomous
 * 
 */
public class LimeLightIntakeCommand extends CommandBase {
    private SwerveDrive m_SwerveDrive;
    private LimeLightUtility m_LimeLight;
    private String m_LimeLightName;
    private double m_goalX;
    private double m_goalY;

    private double m_distance;

    private double m_MaxStrafe = 1.0; //TODO tune this value on the robot. Tune PID value first and set this value as a hard stop to prevent outlying data
    
    private double m_throttle;
    private double m_strafe;

    private static double kDt = 0.0;

    private final TrapezoidProfile.Constraints m_constraints =
        new TrapezoidProfile.Constraints(0.5, 0.5);
    private TrapezoidProfile.State m_goal;
    private TrapezoidProfile.State m_setpoint;
    private TrapezoidProfile profile;

    PIDController pid = new PIDController(0.0, 0.0, 0.0); //(0.01, 0.0, 0.0); //TODO tune this value


    /**
     * <h3>LimeLightIntakeCommand</h3>
     * 
     * Uses game piece detection and a Trapezoidal Profile to smoothly and accurately intake a cube during autonomous
     * 
     * @param swerveDrive Swerve Drive
     * @param limeLight LimeLightUtility
     * @param limeLightName The name of the LimeLight so we can identify which camera we should be using
     * @param goalX X position of the desired target in meters
     * @param goalY Y position of the desired target in meters
     * 
     */
    public LimeLightIntakeCommand(SwerveDrive swerveDrive, LimeLightUtility limeLight, String limeLightName, double goalX, double goalY) {
        m_SwerveDrive = swerveDrive;
        m_LimeLight = limeLight;
        m_LimeLightName = limeLightName;
        m_goalX = goalX;
        m_goalY = goalY;
        addRequirements(m_SwerveDrive);
    }

    @Override
    public void initialize() { 
        m_distance = Math.sqrt( //Uses the Pythagorean Theorem to calculate the total distance to the target
            Math.pow(
                Math.abs(m_goalX - m_SwerveDrive.getPose().getX()), 
                2.0
            )
            +
            Math.pow(
                Math.abs(m_goalY - m_SwerveDrive.getPose().getY()),
                2.0
            )
        );

        m_goal = new TrapezoidProfile.State(m_distance, 0);
        m_setpoint = new TrapezoidProfile.State(0,0);
        profile = new TrapezoidProfile(m_constraints, m_goal, m_setpoint);
    }

    @Override
    public void execute() {
        m_strafe = MathUtil.clamp(pid.calculate(m_LimeLight.get_tx(m_LimeLightName), 0.0), -m_MaxStrafe, m_MaxStrafe);
        Logger.getInstance().recordOutput("m_strafe", m_strafe);
        m_setpoint = profile.calculate(kDt);
        m_throttle = m_setpoint.velocity;
        kDt += 0.02;

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
