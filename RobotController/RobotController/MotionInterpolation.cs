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
            Matrix currentTcpWorldPosition = param.motionTester.CurrentTarget.WorldTargetMatrix;
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
            robotList[robot].currentDistanceToGoal = (robotList[robot].currentTarget.WorldTargetMatrix.GetP() - robot.RobotController.ToolCenterPoint.GetP()).Length;
            return robotList[robot].currentDistanceToGoal;
        }

        private IMotionTarget CreateIMotionTargetForJointAngleConfiguration(IRobot robot, ref Dictionary<IRobot, RobotParameters> robotList, double[] jointAngleCollection, MotionType motionType, double cartesianSpeed)
        {
            IMotionTarget motionTarget = robot.RobotController.CreateTarget();

            motionTarget.MotionType = motionType;
            motionTarget.SetAllJointValues(jointAngleCollection);
            motionTarget.UseJointValues = true;
            motionTarget.IsContinuous = true;
            
            SetSpeedInMotionTarget(robot, ref robotList, ref motionTarget);

            return motionTarget;
        }

        private void SetSpeedInMotionTarget(IRobot robot, ref Dictionary<IRobot, RobotParameters> robotList, ref IMotionTarget motionTarget)
        {
            RobotParameters param = robotList[robot];
            if (motionTarget.MotionType == MotionType.Joint)
            {
                // JointSpeed Value from 0-100
                if (param.allowedCartesianSpeed > motionTarget.CartesianSpeed)
                {
                    motionTarget.JointSpeed = param.maxCartesianSpeed / param.allowedCartesianSpeed;
                }
                else
                {
                    motionTarget.JointSpeed = param.maxCartesianSpeed / motionTarget.CartesianSpeed;
                }
                //ms.AppendMessage("SetJoint Speed for motion to: " + motionTarget.JointSpeed, MessageLevel.Warning);
            }
            else if (motionTarget.MotionType == MotionType.Linear)
            {
                if (param.allowedCartesianSpeed > motionTarget.CartesianSpeed)
                {
                    motionTarget.CartesianSpeed = motionTarget.CartesianSpeed;
                }
                else
                {
                    motionTarget.CartesianSpeed = param.allowedCartesianSpeed;
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
            RobotParameters param = robotList[robot];

            //ms.AppendMessage("Preparing motion at simulation time: " + app.Simulation.Elapsed, MessageLevel.Warning);
            //Clear existing interpolations
            param.motionInterpolator.ClearTargets();

            //Add current position as start for interpolation and adjust speed
            IMotionTarget startTarget = robot.RobotController.CreateTarget();
            SetSpeedInMotionTarget(robot, ref robotList, ref startTarget);
            startTarget.MotionType = MotionType.Linear;
            startTarget.IsContinuous = true;
            param.motionInterpolator.AddTarget(startTarget);

            //Add current target as end for interpolation after speed adjustment
            SetSpeedInMotionTarget(robot, ref robotList, ref robotList[robot].currentTarget);
            param.currentTarget.MotionType = MotionType.Linear;
            param.currentTarget.IsContinuous = true;
            param.motionInterpolator.AddTarget(param.currentTarget);

            param.currentMotionStartTime = simulationTimeElapsed;
            //ms.AppendMessage("CurrentMotionStartTime set to: " + robotList[robot].currentMotionStartTime, MessageLevel.Warning);
            param.currentMotionEndTime = simulationTimeElapsed + param.motionInterpolator.GetCycleTimeAt(robotList[robot].motionInterpolator.Targets.Count - 1);
            //ms.AppendMessage("CurrentMotionEndTime set to: " + robotList[robot].currentMotionEndTime, MessageLevel.Warning);
            //ms.AppendMessage("Finished motion preparing at simulation time: " + app.Simulation.Elapsed, MessageLevel.Warning);
        }

        public void InterpolatePlannedMotion(IRobot robot, ref Dictionary<IRobot, RobotParameters> robotList, double simulationTimeElapsed)
        {
            RobotParameters param = robotList[robot];
            if (param.motionPlan != null && param.motionPlan.getLastResult() != null)
            {
                //ms.AppendMessage("GetCycleTime at " + motionInterpolator.Targets.Count + ": " + motionInterpolator.GetCycleTimeAt(motionInterpolator.Targets.Count-1), MessageLevel.Warning);

                if (param.currentTarget == null && param.motionList.Count > 0)
                {
                    param.currentTarget = param.motionList.First().Value;
                    PrepareNextMotion(robot, ref robotList, simulationTimeElapsed);
                }
                double motionStartTime = param.currentMotionStartTime;
                //ms.AppendMessage("CycleTime:" + app.Simulation.Elapsed, MessageLevel.Warning);
                
                if ((simulationTimeElapsed-motionStartTime) < param.currentMotionEndTime && CalculateDistanceToGoal(robot, ref robotList) > 0.01)
                {
                    //Now interpolate until currentTarget is reached
                    IMotionTarget refMotionTarget = robot.RobotController.CreateTarget();
                    
                    param.motionInterpolator.Interpolate((simulationTimeElapsed - motionStartTime), ref refMotionTarget);
                    //ms.AppendMessage("Executing motion with speed:" + refMotionTarget.CartesianSpeed, MessageLevel.Warning);

                    param.motionTester.CurrentTarget = refMotionTarget;
                }
                else
                {
                    //Get next target from list - if there are targets left
                    if (param.motionList.IndexOfValue(param.currentTarget) + 1 < param.motionList.Count)
                    {
                        param.currentTarget = param.motionList.ElementAt(param.motionList.IndexOfValue(param.currentTarget) + 1).Value;
                        PrepareNextMotion(robot, ref robotList, simulationTimeElapsed);

                    }
                    else
                    {
                        IBooleanSignal movementFinished = (IBooleanSignal)robot.Component.FindBehavior("MovementFinished");
                        movementFinished.Value = true;
                        param.currentTarget = null;
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
            RobotParameters param = robotList[robot];
            if (param.motionInterpolator == null)
            {
                param.motionInterpolator = robot.RobotController.CreateMotionInterpolator();
            }
            else
            {
                param.motionInterpolator.ClearTargets();
            }

            if (param.motionPlan.getLastResult() != null)
            {
                param.motionList.Clear();

                //Converting Joint Angle Configuration to IMotionTargets
                foreach (VectorOfDouble jointAngleCollection in robotList[robot].motionPlan.getLastResult())
                {
                    //TODO: Make it flexible to allow convertion to PTP IMotionTargets as well 
                    IMotionTarget motionTarget = CreateIMotionTargetForJointAngleConfiguration(robot, ref robotList, KukaSorted(jointAngleCollection), MotionType.Linear, param.maxCartesianSpeed);
                    param.motionInterpolator.AddTarget(motionTarget);
                }

                double endTime = simulationTimeElapsed + robotList[robot].motionInterpolator.GetCycleTimeAt(robotList[robot].motionPlan.getLastResult().Count - 1);
                

                //Precompute the IMotionTargets based on the sampling interval
                for (double x = 0; x <= endTime; x += samplingInterval)
                {
                    IMotionTarget refMotionTarget = robot.RobotController.CreateTarget();
                    param.motionInterpolator.Interpolate(x, ref refMotionTarget);
                    double[] jointValues = refMotionTarget.GetAllJointValues();
                    param.motionList.Add(x, refMotionTarget);
                }

                ms.AppendMessage("Created " + param.motionList.Count + " motionTargets for Interpolation", MessageLevel.Warning);

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
