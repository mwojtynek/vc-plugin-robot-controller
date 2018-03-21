using System;

//using SpeedAndSeparationMonitoring;

using Caliburn.Micro;
using VisualComponents.Create3D;
using Rosi.Components.Sensors;

using RosiTools.Debugger;
using SpeedAndSeparationMonitoring;
using RosiTools.Printer;
using RosiTools.Collector;
using System.Collections.Generic;
using Rosi.Components.Sensors.LaserScanner;

namespace CustomController
{

    // SSM Part
    // maximumSpeed kontrolliert alles!

    public partial class CustomController
    {
        private bool useSSM = true;

        // Seperation Values
        private double t_reaction = 0.1;
        private double t_robot_stop = 0.31;
        private double d_intrusion = 160;
        private double s_human = 160;
        private double s_robot = 0.04;
        SeparationCalculator separationCalculator;
        private double separationDistance = 100.0;

        // Speed Values
        private double m_robot = 30;
        private double m_payload = 0;
        SpeedCalculator speedCalculator;

        public void InitSSM(Object sender, EventArgs data)
        {
            separationCalculator = new SeparationCalculator(t_reaction, t_robot_stop, d_intrusion, s_human, s_robot);
            speedCalculator = new SpeedCalculator(m_robot, m_payload);
            VisualizeSeperationDistance(400.0);

            IList<CollectorClass> scanners = IoC.Get<ICollectorManager>().getInstances("LaserScanner");

            foreach (LaserScannerCollector scanner in scanners)
            {
                scanner.OnHumanDetected += HumanDetected;
                scanner.OnHumanLost += HumanLost;
            }
        }

        public void KillSSM(Object sender, EventArgs data)
        {
            separationCalculator = null;
            speedCalculator = null;

            IList<CollectorClass> scanners = IoC.Get<ICollectorManager>().getInstances("LaserScanner");

            foreach (LaserScannerCollector scanner in scanners)
            {
                scanner.OnHumanDetected -= HumanDetected;
                scanner.OnHumanLost -= HumanLost;
            }
        }

        public void HumanDetected(Object sender, LaserScannerHumanDetectedEventArgs data)
        {
            IRobot robot = manip.robot;
            if (robot.Equals(data.Robot))
            {
                Vector3 tcpSpeedAsVector3 = new Vector3(tcpSpeed[0], tcpSpeed[1], tcpSpeed[2]);

                Vector3 tcpPosition = robot.RobotController.ToolCenterPoint.GetP();
                Vector3 humanPosition = data.HumanPosition;

                double speedTowardsHuman = CalculateSpeedTowardsHuman(tcpSpeedAsVector3, tcpPosition, humanPosition);

                separationDistance = separationCalculator.GetSeparationDistance(data.MoveSpeed, speedTowardsHuman);
                double allowedSpeed = speedCalculator.GetAllowedVelocity(BodyPart.Chest, data.MoveSpeed, 1);

                double distance = (tcpPosition - humanPosition).Length;
                maximumSpeed = distance < separationDistance ? 0 : allowedSpeed;
            }
        }

        public void HumanLost(Object sender, LaserScannerHumanLostEventArgs data)
        {
            if (manip.robot.Equals(data.Robot))
            {
                maximumSpeed = demandedSpeed;
            }
        }

        private double CalculateSpeedTowardsHuman(Vector3 tcpSpeed, Vector3 robotPosition, Vector3 humanPosition)
        {
            Vector3 positionOffset = humanPosition - robotPosition;
            Vector3 normalizedOffset = positionOffset / positionOffset.Length;
            Vector3 normalizedSpeed = tcpSpeed;
            if (tcpSpeed.Length != 0)
            {
                normalizedSpeed /=  tcpSpeed.Length;
            }

            double vecProduct = Vector3.Dot(normalizedOffset, normalizedSpeed);
            double speedTowardsHuman = vecProduct * tcpSpeed.Length;

            return speedTowardsHuman;
        }

        /// <summary>
        /// Visualization of Separation Distance. A cylinder feature is added to the robot if not already present.
        /// Initial values are set.
        /// </summary>
        /// <param name="robot"></param>The robot for which the separation distance should be visualized.
        /// <param name="initialRadius"></param>The initial radius for the cylinder.
        private void VisualizeSeperationDistance(double initialRadius)
        {
            IRobot robot = manip.robot;
            if (app.World.FindComponent("SeparationVisualization_" + robot.Name) == null)
            {
                ISimComponent component = app.World.CreateComponent("SeparationVisualization_" + robot.Name);
                component.CreateProperty(typeof(Double), PropertyConstraintType.NotSpecified, "SeparationDistance");
                ISimNode node = robot.Component.FindNode("mountplate");

                Matrix matrix = component.TransformationInReference;
                matrix.SetP(new Vector3(node.TransformationInWorld.Px, node.TransformationInWorld.Py, 201));

                component.TransformationInReference = matrix;

                ICylinderFeature seperationVisualization = component.RootNode.RootFeature.CreateFeature<ICylinderFeature>();
                // true would remove the top and bottom of the cylinder, but backfaces of the inside of the cylinder are not rendered
                //seperationVisualization.GetProperty("Caps").Value = false; 
                seperationVisualization.GetProperty("Height").Value = "1.0";
                seperationVisualization.GetProperty("Sections").Value = "36.0";
                seperationVisualization.GetProperty("Radius").Value = initialRadius.ToString();
                seperationVisualization.GetProperty("Material").Value = app.FindMaterial("transp_yellow", false);
                seperationVisualization.SetName("SeparationVisualization");
            }
        }

        /// <summary>
        /// Simple function to update the size of the cylinder which visualizes the current separation distance.
        /// </summary>
        /// <param name="robot"></param>The robot for which the update should be made.
        private void UpdateVisualizationDistance()
        {
            IRobot robot = manip.robot;
            if (useSSM)
            {
                ISimComponent comp = app.World.FindComponent("SeparationVisualization_" + robot.Name);
                ISimNode node = robot.Component.FindNode("mountplate");

                Matrix matrix = comp.TransformationInReference;
                matrix.SetP(new Vector3(node.TransformationInWorld.Px, node.TransformationInWorld.Py, 201));

                comp.TransformationInReference = matrix;

                IFeature cylinder = comp.FindFeature("SeparationVisualization");
                if (separationDistance <= 100.0)
                {
                    cylinder.GetProperty("Radius").Value = "100";
                    comp.GetProperty("SeparationDistance").Value = "100";
                }
                else
                {
                    cylinder.GetProperty("Radius").Value = separationDistance.ToString();
                    comp.GetProperty("SeparationDistance").Value = separationDistance;
                }
                cylinder.Rebuild();
            }
            else
            {
                Printer.print("UpdateVisualizationDistance: Failed to find robot component!");
            }
        }
    }
}
