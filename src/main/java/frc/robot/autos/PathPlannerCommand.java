package frc.robot.autos;

import frc.robot.subsystems.SwerveDrive;
import frc.robot.utilities.FieldCentricOffset;
import frc.robot.utilities.SwerveAutoBuilder;

import java.util.List;
import java.util.Map;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/*
 * <h3>PathPlannerCommand<h3>
 * 
 */
public class PathPlannerCommand extends SequentialCommandGroup {

    private static final double MAX_ACCELERATION = 2.5;
    private static final double MAX_VELOCITY = 1.0;

     /**
      * 
      * Adding path constraints and builds auto command
      *
      * @param s_Swerve Swerve drive
      * @param preCommand_1 The command that is run before the path starts
      * @param pathName_1 The name of the first path that is run
      * @param Command_2 A command that is run after the first path ends (optional)
      * @param pathName_2 The second path that is run (optional)
      * @param Command_3 A command that is runa fter the second path ends (optional)
      * @param pathName_3 The third path that is run (optional)
      * @param postCommand_4 The command that is run after the path completes
      * @param eventCommandMap A command that uses strings to return a command that we want to execute at a marker
      */
    public PathPlannerCommand(SwerveDrive s_Swerve, Command preCommand_1, String pathName_1, Command Command_2, String pathName_2, Command Command_3, String pathName_3, Command postCommand_4, Map<String, Command> eventCommandMap) {
        addRequirements(s_Swerve);

        PathConstraints pathConstraints = PathPlanner.getConstraintsFromPath(pathName_1);
        if(pathConstraints == null) {
            pathConstraints = new PathConstraints(MAX_VELOCITY, MAX_ACCELERATION);
        }

        // Load Path Group trajectory to follow. All units in meters.
        List<PathPlannerTrajectory> loadPathGroup = PathPlanner.loadPathGroup(pathName_1, 
             false, pathConstraints);
             
        List<PathPlannerTrajectory> loadPathGroup_2 = null;
        if (pathName_2 != null) {
            loadPathGroup_2 = PathPlanner.loadPathGroup(pathName_2, 
            false, pathConstraints);
        }
        final List<PathPlannerTrajectory> finalLoadPathGroup_2 = loadPathGroup_2;

        List<PathPlannerTrajectory> loadPathGroup_3 = null;
        if (pathName_3 != null) {
            loadPathGroup_3 = PathPlanner.loadPathGroup(pathName_3, 
             false, pathConstraints);
        }
        final List<PathPlannerTrajectory> finalLoadPathGroup_3 = loadPathGroup_3;

        SwerveAutoBuilder autoBuilder = 
            new SwerveAutoBuilder(
                s_Swerve::getPose, //Using Pose Swerve estimator
                s_Swerve::resetOdometry, //pose2D consumer, used to reset odometry at beginning of zero
                SwerveDrive.getSwerveKinematics(),
                new PIDConstants(1.75, 0.0, 0.175), //PID constants to correct for translation error (X and Y)
                //new PIDConstants(1.0, 0.0, 0.0), //PID constants to correct for rotation error (used to create the rotation controller)
                new PIDConstants(1.9, 0.0, 0.19), //PID constants to correct for rotation error (used to create the rotation controller)
                s_Swerve::setSwerveModuleStates,
                eventCommandMap, 
                true, // TODO Should the path be automatically mirrored depending on alliance color
                s_Swerve);

        // Adding a pre command to autonomous ex. autoBalance
        if(preCommand_1 != null) {
            addCommands(preCommand_1);
        }

        // creates a command based on the path group
        Command swerveControllerCommand = autoBuilder.fullAuto(loadPathGroup);
        addCommands(
            // TODO: Use april tags to help set this
            new InstantCommand(() -> FieldCentricOffset.getInstance().setOffset(loadPathGroup.get(0).getInitialHolonomicPose().getRotation().getDegrees())),
            swerveControllerCommand
        );

        if (Command_2 != null) {
            addCommands(Command_2);
        }
        
        if (pathName_2 != null) {
            //creates a second path
            Command swerveControllerCommand_2 = autoBuilder.fullAuto(loadPathGroup_2);
            addCommands(
                // TODO: Use april tags to help set this
                new InstantCommand(() -> FieldCentricOffset.getInstance().setOffset(finalLoadPathGroup_2.get(0).getInitialHolonomicPose().getRotation().getDegrees())),
                swerveControllerCommand_2
            );
        }

        if (Command_3 != null) {
            addCommands(Command_3);
        }

        if (pathName_3 != null) {
            //creates a second path
            Command swerveControllerCommand_3 = autoBuilder.fullAuto(loadPathGroup_3);
            addCommands(
                // TODO: Use april tags to help set this
                new InstantCommand(() -> FieldCentricOffset.getInstance().setOffset(finalLoadPathGroup_3.get(0).getInitialHolonomicPose().getRotation().getDegrees())),
                swerveControllerCommand_3
            );
        }

        if (postCommand_4 != null) {
            addCommands(postCommand_4);
        }
    }

    /**
     * 
     * <h3>PathPlannerCommand</h3>
     * 
     * Adds path constraints and builds auto command
     * 
     * @param s_Swerve the required subsystem
     * @param pathName name of path (pathPlanner's path)
     * @param eventCommandMap a command that uses strings to return a command that we want to execute at a marker
     * @param postCommand a command that is run after the path completes
     */
    public PathPlannerCommand(SwerveDrive s_Swerve, String pathName, Map<String, Command> eventCommandMap, Command postCommand) {
            this(s_Swerve, null, pathName, null, null, null, null, postCommand, eventCommandMap);
    }


    /**
     * <h3>PathPlannerCommand</h3>
     * 
     * Adds path constraints and builds auto command
     * 
     * @param s_Swerve the required subsystem
     * @param pathName name of path (pathPlanner's path)
     * @param eventCommandMap a command that uses strings to return a command that we want to execute at a marker
     */
    public PathPlannerCommand(SwerveDrive s_Swerve, String pathName, Map<String, Command> eventCommandMap) {
        this(s_Swerve, pathName, eventCommandMap, null);
    }

    /**
     * 
     * <h3>PathPlannerCommand</h3>
     * 
     * Adds path constraints and builds auto command
     * 
     * @param preCommand a command that is run before the path starts
     * @param s_Swerve the required subsystem
     * @param pathName name of path (pathPlanner's path)
     * @param eventCommandMap a command that uses strings to return a command that we want to execute at a marker
     */
    public PathPlannerCommand(Command preCommand, SwerveDrive s_Swerve, String pathName, Map<String, Command> eventCommandMap) {
        this(s_Swerve, preCommand, pathName, null, null, null, null, null, eventCommandMap);
    }

    public PathPlannerCommand(SwerveDrive s_Swerve, Command preCommand_1, String pathName_1, Command Command_2, String pathName_2, Command Command_3, String pathName_3, Map<String, Command> eventCommandMap) {
        this(s_Swerve, preCommand_1, pathName_1, Command_2, pathName_2, Command_3, pathName_3, null, eventCommandMap);
    }
}