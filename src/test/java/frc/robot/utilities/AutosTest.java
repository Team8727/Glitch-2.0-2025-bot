
package frc.robot.utilities;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.kAllianceInfo.RobotAlliance;
import frc.robot.Constants.kVision;
import frc.robot.subsystems.Autos;

public class AutosTest {

    private NetworkTableLogger logger = new NetworkTableLogger(this.getClass().getName());

    /**
     * This method tests the findClosestReefSide() method in the enum ReefScorePoints to find the closest reef zone, i.e A_B
     * to the robot's current position.
     * </p> IMPORTANT: Must go in a periodic (or any repeating) block to continuously log values. 
     * @param alliance RobotAlliance enum value in Constants.java representing red or blue alliance 
     *      (use ternary operator with Robot.isRedAlliance() to choose enum value)
     * @param robotPose Current robotPose as a Pose2d from PoseEstimator.java as get2dPose()
     */
    public void testFindClosestReefSide(RobotAlliance alliance, Pose2d robotPose) {
        // Logs Pose2d of closest reef side
        logger.logPose2d("Closest Reef Side Pose", 
            new Pose2d(
                Autos.ReefScorePoints
                    .findClosestReefSide(alliance, robotPose)
                        .getPoint(),
                new Rotation2d()).rotateAround(
                    kVision.fieldCenter, 
                    alliance == RobotAlliance.RED_ALLIANCE ? 
                        new Rotation2d(Math.toRadians(180)) : new Rotation2d(Math.toRadians(0))));
    }

}