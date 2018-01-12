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

        public void CalculateCurrentRobotSpeed(IRobot robot, ref Dictionary<IRobot, RobotParameters> robotList, Matrix currentTcpWorldPosition, double tickInterval)
        {
            //Distance between last and current position
            if (!robotList[robot].lastTcpWorldPosition.Equals(Matrix.Zero))
            {
                double distance = (robotList[robot].lastTcpWorldPosition.GetP() - robot.RobotController.ToolCenterPoint.GetP()).Length;
                //[mm/s]
                robotList[robot].currentCartesianSpeed = distance * 1 / tickInterval;
                //ms.AppendMessage("Current Cartesian Speed measured: " + robotList[robot].currentCartesianSpeed, MessageLevel.Warning);
            }

            robotList[robot].lastTcpWorldPosition = currentTcpWorldPosition;
        }

        private double CalculateDistanceToGoal(IRobot robot, ref Dictionary<IRobot, RobotParameters> robotList)
        {
            //Distance between goal positon and current position
            robotList[robot].currentDistanceToGoal = (robotList[robot].currentTarget.WorldTargetMatrix.GetP() - robot.RobotController.ToolCenterPoint.GetP()).Length;
            return robotList[robot].currentDistanceToGoal;
        }

        private IMotionTarget CreateIMotionTargetForJointAngleConfiguration(IRobot robot, ref Dictionary<IRobot, RobotParameters> robotList, double[] jointAngleCollection, MotionType motionType, double cartesianSpeed)
        {
            IMotionTarget motionTarget = robot.RobotController.CreateTarget();

            motionTarget.MotionType = motionType;
            motionTarget.SetAllJointValues(jointAngleCollection);
            motionTarget.UseJointValues = true;
            SetSpeedInMotionTarget(robot, ref robotList, ref motionTarget);

            return motionTarget;
        }

        private void SetSpeedInMotionTarget(IRobot robot, ref Dictionary<IRobot, RobotParameters> robotList, ref IMotionTarget motionTarget)
        {
            if (motionTarget.MotionType == MotionType.Joint)
            {
                // JointSpeed Value from 0-100
                if (robotList[robot].allowedCartesianSpeed > motionTarget.CartesianSpeed)
                {
                    motionTarget.JointSpeed = robotList[robot].maxCartesianSpeed / robotList[robot].allowedCartesianSpeed;
                }
                else
                {
                    motionTarget.JointSpeed = robotList[robot].maxCartesianSpeed / motionTarget.CartesianSpeed;
                }
                //ms.AppendMessage("SetJoint Speed for motion to: " + motionTarget.JointSpeed, MessageLevel.Warning);
            }
            else if (motionTarget.MotionType == MotionType.Linear)
            {
                if (robotList[robot].allowedCartesianSpeed > motionTarget.CartesianSpeed)
                {
                    motionTarget.CartesianSpeed = motionTarget.CartesianSpeed;
                }
                else
                {
                    motionTarget.CartesianSpeed = robotList[robot].allowedCartesianSpeed;
                }
                //ms.AppendMessage("SetCartesian Speed for motion to: " + motionTarget.CartesianSpeed, MessageLevel.Warning);
            }
        }

        /// <summary>
        /// This method helps to update the motion speeds in the motion interpolator directly.
        /// First cleares the current motionInterpolator instance of the robot.
        /// Then takes the current robot position transforms it in an IMotion target, adjusts the speed and adds it to the motionInterpolator.
        /// Afterwards takes the current target from robot parameters, updates the speed and adds it to the motionInterpolator.
        /// Then the new end time of the motion is calculated and set in the robot parameters.
        /// </summary>
        /// <param name="robot"></param>
        private void PrepareNextMotion(IRobot robot, ref Dictionary<IRobot, RobotParameters> robotList, double simulationTimeElapsed)
        {
            //ms.AppendMessage("Preparing motion at simulation time: " + app.Simulation.Elapsed, MessageLevel.Warning);
            //Clear existing interpolations
            robotList[robot].motionInterpolator.ClearTargets();

            //Add current position as start for interpolation and adjust speed
            IMotionTarget startTarget = robot.RobotController.CreateTarget();
            SetSpeedInMotionTarget(robot, ref robotList, ref startTarget);
            startTarget.MotionType = MotionType.Linear;
            robotList[robot].motionInterpolator.AddTarget(startTarget);

            //Add current target as end for interpolation after speed adjustment
            SetSpeedInMotionTarget(robot, ref robotList, ref robotList[robot].currentTarget);
            robotList[robot].currentTarget.MotionType = MotionType.Linear;
            robotList[robot].motionInterpolator.AddTarget(robotList[robot].currentTarget);

            robotList[robot].currentMotionStartTime = simulationTimeElapsed;
            //ms.AppendMessage("CurrentMotionStartTime set to: " + robotList[robot].currentMotionStartTime, MessageLevel.Warning);
            robotList[robot].currentMotionEndTime = simulationTimeElapsed + robotList[robot].motionInterpolator.GetCycleTimeAt(robotList[robot].motionInterpolator.Targets.Count - 1);
            //ms.AppendMessage("CurrentMotionEndTime set to: " + robotList[robot].currentMotionEndTime, MessageLevel.Warning);
            //ms.AppendMessage("Finished motion preparing at simulation time: " + app.Simulation.Elapsed, MessageLevel.Warning);
        }

        public void InterpolatePlannedMotion(IRobot robot, ref Dictionary<IRobot, RobotParameters> robotList, double simulationTimeElapsed)
        {
            if (robotList[robot].motionPlan != null && robotList[robot].motionPlan.getLastResult() != null)
            {
                //ms.AppendMessage("GetCycleTime at " + motionInterpolator.Targets.Count + ": " + motionInterpolator.GetCycleTimeAt(motionInterpolator.Targets.Count-1), MessageLevel.Warning);
                
                if (robotList[robot].currentTarget == null && robotList[robot].motionList.Count > 0)
                {
                    robotList[robot].currentTarget = robotList[robot].motionList.First().Value;
                    PrepareNextMotion(robot, ref robotList, simulationTimeElapsed);
                }

                //ms.AppendMessage("CycleTime:" + app.Simulation.Elapsed, MessageLevel.Warning);

                if (simulationTimeElapsed < robotList[robot].currentMotionEndTime && CalculateDistanceToGoal(robot, ref robotList) > 0.01)
                {
                    //Now interpolate until currentTarget is reached
                    IMotionTarget refMotionTarget = robot.RobotController.CreateTarget();

                    robotList[robot].motionInterpolator.Interpolate(simulationTimeElapsed - robotList[robot].currentMotionStartTime, ref refMotionTarget);
                    //ms.AppendMessage("Executing motion with speed:" + refMotionTarget.CartesianSpeed, MessageLevel.Warning);

                    robotList[robot].motionTester.CurrentTarget = refMotionTarget;
                }
                else
                {
                    //Get next target from list - if there are targets left
                    if (robotList[robot].motionList.IndexOfValue(robotList[robot].currentTarget) + 1 < robotList[robot].motionList.Count)
                    {
                        robotList[robot].currentTarget = robotList[robot].motionList.ElementAt(robotList[robot].motionList.IndexOfValue(robotList[robot].currentTarget) + 1).Value;
                        PrepareNextMotion(robot, ref robotList, simulationTimeElapsed);

                    }
                    else
                    {
                        IBooleanSignal movementFinished = (IBooleanSignal)robot.Component.FindBehavior("MovementFinished");
                        movementFinished.Value = true;
                        robotList[robot].currentTarget = null;
                    }
                }

            }
        }

        /// <summary>
        /// Takes the current motion plan and uses the interpolator to generate IMotionTargets for a defined interval:
        /// e.g. an IMotionTarget for every second of the complete motion
        /// </summary>
        /// <param name="robot"></param>
        public void CalculateInterpolation(IRobot robot, ref Dictionary<IRobot, RobotParameters> robotList, double samplingInterval, double simulationTimeElapsed)
        {

            if (robotList[robot].motionInterpolator == null)
            {
                robotList[robot].motionInterpolator = robot.RobotController.CreateMotionInterpolator();
            }

            if (robotList[robot].motionPlan.getLastResult() != null)
            {
                //Converting Joint Angle Configuration to IMotionTargets
                foreach (VectorOfDouble jointAngleCollection in robotList[robot].motionPlan.getLastResult())
                {
                    //TODO: Make it flexible to allow convertion to PTP IMotionTargets as well 
                    IMotionTarget motionTarget = CreateIMotionTargetForJointAngleConfiguration(robot, ref robotList, KukaSorted(jointAngleCollection), MotionType.Linear, robotList[robot].maxCartesianSpeed);
                    robotList[robot].motionInterpolator.AddTarget(motionTarget);
                }

                double endTime = simulationTimeElapsed + robotList[robot].motionInterpolator.GetCycleTimeAt(robotList[robot].motionPlan.getLastResult().Count - 1);

                ms.AppendMessage("StartTime: " + simulationTimeElapsed + " , EndTime: " + endTime, MessageLevel.Warning);

                //Precompute the IMotionTargets based on the sampling interval
                for (double x = simulationTimeElapsed; x <= endTime; x = x + samplingInterval)
                {
                    IMotionTarget refMotionTarget = robot.RobotController.CreateTarget();
                    robotList[robot].motionInterpolator.Interpolate(x, ref refMotionTarget);
                    robotList[robot].motionList.Add(x, refMotionTarget);
                }

                ms.AppendMessage("Created " + robotList[robot].motionList.Count + " motionTargets for Interpolation", MessageLevel.Warning);
            }
            else
            {
                ms.AppendMessage("Something went wrong in motion planning, no results were available", MessageLevel.Warning);
            }
        }


        private double[] KukaSorted(VectorOfDouble jointAngleCollection)
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
