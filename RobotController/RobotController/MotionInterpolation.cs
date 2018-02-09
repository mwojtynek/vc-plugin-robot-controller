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

        public void CalculateCurrentRobotSpeed(IRobot robot, ref Dictionary<IRobot, RobotParameters> robotList, double tickInterval, Vector3 humanWorldPosition)
        {
            
            RobotParameters param = robotList[robot];
            Vector3 currentTcpWorldPosition = robot.Component.TransformationInWorld.GetP() + robot.RobotController.ToolCenterPoint.GetP(); //robot.RobotController.GetMotionTester().CurrentTarget.WorldTargetMatrix;
            //Distance between last and current position
            if (!param.lastTcpWorldPosition.Equals(Matrix.Zero))
            {
                double distance = (param.lastTcpWorldPosition - currentTcpWorldPosition).Length;
                if (distance != 0.0)
                {
                    //[mm/s]
                    double cartesianSpeed = distance / tickInterval;

                    double dx = currentTcpWorldPosition.X - param.lastTcpWorldPosition.X;
                    double dy = currentTcpWorldPosition.Y - param.lastTcpWorldPosition.Y;
                    double dz = currentTcpWorldPosition.Z - param.lastTcpWorldPosition.Z;

                    double d = Math.Sqrt((dx * dx) + (dy * dy) + (dz * dz));

                    double vx = dx / d * cartesianSpeed;
                    double vy = dy / d * cartesianSpeed;
                    double vz = dz / d * cartesianSpeed;

                    double vxy = Math.Sqrt((vx * vx) + (vy * vy));

                    double alpha = Math.Atan2((humanWorldPosition.Y - currentTcpWorldPosition.Y), (humanWorldPosition.X - currentTcpWorldPosition.X)); 

                    robot.Component.GetProperty("ArrowAngleZ").Value = alpha * (180 / Math.PI); //param.angleToHuman * (180 / Math.PI);
                                                                                                // Amount of speed of the robot that is directed towards the human
                    double vHuman = (vx * Math.Cos(alpha)) + (vy * Math.Sin(alpha));
                    //ms.AppendMessage("Vx: " + vx + ", Vy: " + vy + ", VHuman: " + vHuman + "Alpha: " + alpha, MessageLevel.Error);

                    if (Math.Abs(vHuman - param.currentCartesianSpeed) >= 100)
                    {
                        // Do nothing speed calculation seems to be wrong, we don't want spikes
                    }
                    else
                    {
                        param.currentCartesianSpeed = vHuman;
                    }
                    
                }


                
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
