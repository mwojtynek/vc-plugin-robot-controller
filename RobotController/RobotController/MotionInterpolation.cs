using Caliburn.Micro;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using VisualComponents.Create3D;
using VisualComponents.UX.Shared;

namespace RobotController
{
    class MotionInterpolation
    {
        private IMessageService ms = null;

        public MotionInterpolation()
        {
            ms = IoC.Get<IMessageService>();
        }

        public void CalculateCurrentRobotSpeed(IRobot robot, ref Dictionary<IRobot, RobotParameters> robotList, double tickInterval)
        {
            RobotParameters param = robotList[robot];
            Matrix currentTcpWorldPosition = robot.RobotController.GetMotionTester().CurrentTarget.WorldTargetMatrix;
            //Distance between last and current position
            if (!param.lastTcpWorldPosition.Equals(Matrix.Zero))
            {
                double distance = (param.lastTcpWorldPosition.GetP() - currentTcpWorldPosition.GetP()).Length;
                //[mm/s]
                param.currentCartesianSpeed = distance * 1 / tickInterval;
                //ms.AppendMessage("Current Cartesian Speed measured: " + robotList[robot].currentCartesianSpeed, MessageLevel.Warning);
            }
            param.lastTcpWorldPosition = currentTcpWorldPosition;
        }

        private double CalculateDistanceToGoal(IRobot robot, ref Dictionary<IRobot, RobotParameters> robotList)
        {
            //Distance between goal positon and current position
            robotList[robot].currentDistanceToGoal = (robot.RobotController.GetMotionTester().CurrentTarget.WorldTargetMatrix.GetP() - robot.RobotController.ToolCenterPoint.GetP()).Length;
            return robotList[robot].currentDistanceToGoal;
        }
        
        public static double[] KukaSorted(VectorOfDouble jointAngleCollection)
        {
            double[] firstJointAngleCollectionSorted = new double[7];

            //VectorOfDouble firstJointAngleCollectionSorted = new VectorOfDouble(jointAngleCollection.Count);
            firstJointAngleCollectionSorted[0] = jointAngleCollection.ElementAt(0); //A1
            firstJointAngleCollectionSorted[1] = jointAngleCollection.ElementAt(1); //A2
            firstJointAngleCollectionSorted[2] = jointAngleCollection.ElementAt(3); //A3
            firstJointAngleCollectionSorted[3] = jointAngleCollection.ElementAt(4); //A4
            firstJointAngleCollectionSorted[4] = jointAngleCollection.ElementAt(5); //A5
            firstJointAngleCollectionSorted[5] = jointAngleCollection.ElementAt(6); //A6
            firstJointAngleCollectionSorted[6] = jointAngleCollection.ElementAt(2); //A7

            return firstJointAngleCollectionSorted;
        }
    }
}
