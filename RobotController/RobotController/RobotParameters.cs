using SpeedAndSeparationMonitoring;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using VisualComponents.Create3D;

namespace RobotController
{
    public class RobotParameters
    {
        public MotionPlan motionPlan = null;
        public double currentCartesianSpeed = 0.0;
        public double currentSeperationDistance = 0.0;
        public double oldSeparationDistance = 0.0;
        public double allowedCartesianSpeed = 0.0;
        public double maxCartesianSpeed = 0.0;
        public SpeedCalculator speedCalculator = null;
        public SeparationCalculator seperationCalculator = null;
        public double currentMotionStartTime = 0.0;
        public String payloadOnFinishMovement = null;

        public Vector3 lastTcpWorldPosition = new Vector3();
        public double closestDistanceToHuman = 100.0;
        public Vector3 closestHumanWorldPosition = new Vector3();
        public double currentDistanceToGoal = 1000.0;
        public double angleToHuman = 0.0;
        
        // Properties below this line are relevant for the setup of the motion plan and the kinematic chain of it only.
        public static String UrdfFile = "S:\\git\\rosi.plugin.pathplanner\\robot_descriptions\\urdf\\lbr_iiwa_14_r820.urdf";
        public static String KinStart = "base_link";
        public static String KinEnd = "tool0";
        public static String obstacleModelFile = "S:\\git\\rosi.plugin.pathplanner\\cage-models\\Robot_Scene_Scaled.stl";
    
        public enum Interpolation_Method
        {
            MOTION_TARGET_CHUNKING, // Hendrik
            TIME_MANIPULATION // Julius
        }
        public static Interpolation_Method InterpolationMethod = Interpolation_Method.TIME_MANIPULATION;
	}
}
