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
    public partial class CustomController
    {
        private bool useSSM = true;

        // Seperation Values
        private double t_reaction = 0.1;
        private double t_robot_stop = 0.31;
        private double d_intrusion = 160;
        private double s_human = 160;
        private double s_robot = 0.04;
        private SeparationCalculator separationCalculator;
        private double separationDistance;
        private double lastSeparationDistance;
        private const double minimumSeparationDistance = 300.0;
        private const double allowedSeparationDistanceChange = 10.0;
        private const double cooldownFactor = 1.5;

        // Statistics
        private double humanDistance = 0.0;
        private double humanAngle = 0.0;

        // Speed Values
        private double m_robot = 30;
        private double m_payload = 0;
        private SpeedCalculator speedCalculator;

        public void InitSSM(Object sender, EventArgs data)
        {
            separationCalculator = new SeparationCalculator(t_reaction, t_robot_stop, d_intrusion, s_human, s_robot);
            speedCalculator = new SpeedCalculator(m_robot, m_payload);
            separationDistance = minimumSeparationDistance;
            lastSeparationDistance = minimumSeparationDistance;
            CreateSeparationDistanceVisualization(separationDistance);

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
                Vector3 relativeTCP = robot.RobotController.ToolCenterPoint.GetP();
                Vector3 robotPosition = robot.RobotController.RobotWorldNode.TransformationInWorld.GetP();
                Vector3 tcpPosition = robotPosition + relativeTCP;
                Vector3 humanPosition = data.HumanPosition;

                // Update statistics
                humanDistance = (tcpPosition - humanPosition).Length;
                humanAngle = data.Angle;

                Vector3 tcpSpeedAsVector3 = new Vector3(tcpSpeed[0], tcpSpeed[1], tcpSpeed[2]);
                double robotSpeedTowardsHuman = CalculateSpeedTowardsHuman(tcpSpeedAsVector3, tcpPosition, humanPosition);
                double humanSpeedTowardsRobot = data.MoveSpeed;

                UpdateSeparationDistance(humanSpeedTowardsRobot, robotSpeedTowardsHuman);
                UpdateAllowedSpeed(humanSpeedTowardsRobot, tcpPosition, humanPosition);
            }
        }

        public void HumanLost(Object sender, LaserScannerHumanLostEventArgs data)
        {
            if (manip.robot.Equals(data.Robot))
            {
                allowedSpeed = demandedSpeed;
            }
        }

        /// <summary>
        /// Updates the separation distance and adds smoothness
        /// </summary>
        /// <param name="humanSpeedTowardsRobot"> [mm/s] </param>
        /// <param name="robotSpeedTowardsHuman"> [mm/s] </param>
        private void UpdateSeparationDistance(double humanSpeedTowardsRobot, double robotSpeedTowardsHuman)
        {
            separationDistance = separationCalculator.GetSeparationDistance(humanSpeedTowardsRobot, robotSpeedTowardsHuman) * 2.0;

            double diff = separationDistance - lastSeparationDistance;

            // TODO Smoothness allowed when separation distance has to grow?
            if(diff > allowedSeparationDistanceChange)
            {
                separationDistance = lastSeparationDistance + allowedSeparationDistanceChange;
            } else if (diff < -allowedSeparationDistanceChange)
            {
                separationDistance = lastSeparationDistance - allowedSeparationDistanceChange;
            }

            separationDistance = Math.Max(separationDistance, minimumSeparationDistance);
            lastSeparationDistance = separationDistance;
        }

        /// <summary>
        /// Updates allowed speed: Robot gets slowed down, when human is close to the separation distance
        /// </summary>
        /// <param name="humanSpeedTowardsRobot"> [mm/s] </param>
        /// <param name="tcpPosition"></param>
        /// <param name="humanPosition"></param>
        private void UpdateAllowedSpeed(double humanSpeedTowardsRobot, Vector3 tcpPosition, Vector3 humanPosition)
        {
            // TODO result from calculation is acutally not used
            double allowedSpeedDuringTransientContact = speedCalculator.GetAllowedVelocity(BodyPart.Chest, humanSpeedTowardsRobot, 1);

            double distance = CalculateDistance2D(tcpPosition, humanPosition);

            if (distance < separationDistance)
            {
                this.allowedSpeed = 0.0;
            }
            else if (this.allowedSpeed != 0.0 && distance < separationDistance * cooldownFactor)
            {
                this.allowedSpeed *= 0.9;
            }
            else
            {
                this.allowedSpeed = demandedSpeed;
            }
        }

        private double CalculateSpeedTowardsHuman(Vector3 tcpSpeed, Vector3 robotPosition, Vector3 humanPosition)
        {
            Vector3 positionOffset = humanPosition - robotPosition;
            Vector3 normalizedOffset = positionOffset / positionOffset.Length;
            Vector3 normalizedSpeed = tcpSpeed;

            if (tcpSpeed.Length != 0)
            {
                normalizedSpeed /= tcpSpeed.Length;
            }

            double vecProduct = Vector3.Dot(normalizedOffset, normalizedSpeed);
            double speedTowardsHuman = vecProduct * tcpSpeed.Length;

            return speedTowardsHuman;
        }

        private double CalculateDistance2D(Vector3 v1, Vector3 v2)
        {
            double deltaX = v1.X - v2.X;
            double deltaY = v1.Y - v2.Y;
            return Math.Sqrt(deltaX * deltaX + deltaY * deltaY);
        }

        /************** VISUALIZATION **************/

        /// <summary>
        /// Visualization of Separation Distance. A cylinder feature is added to the robot if not already present.
        /// Initial values are set.
        /// </summary>
        /// <param name="robot"></param>The robot for which the separation distance should be visualized.
        /// <param name="initialRadius"></param>The initial radius for the cylinder.
        private void CreateSeparationDistanceVisualization(double initialRadius)
        {
            string name = "SeparationVisualization_" + component.Name;
            ISimComponent visualizationComponent = app.World.FindComponent(name);
            if (visualizationComponent != null)
            {
                app.World.DeleteComponent(visualizationComponent);
            }

            visualizationComponent = app.World.CreateComponent(name);

            // Create necessary properties
            IDoubleProperty separationDistanceProperty = (IDoubleProperty) visualizationComponent.CreateProperty(typeof(Double), PropertyConstraintType.NotSpecified, "SeparationDistance");
            separationDistanceProperty.Value = initialRadius;

            IDoubleProperty cooldownFactorProperty = (IDoubleProperty) visualizationComponent.CreateProperty(typeof(Double), PropertyConstraintType.NotSpecified, "CooldownFactor");
            cooldownFactorProperty.Value = cooldownFactor;

            // calculate and set position of the separation cylinder
            ISimNode node = component.FindNode("mountplate");
            Matrix matrix = visualizationComponent.TransformationInReference;
            matrix.SetP(new Vector3(node.TransformationInWorld.Px, node.TransformationInWorld.Py, 201));
            visualizationComponent.TransformationInReference = matrix;

            // manipulate properties
            ICylinderFeature separationVisualization = visualizationComponent.RootNode.RootFeature.CreateFeature<ICylinderFeature>();
            separationVisualization.SetName("SeparationVisualization");

            SetDoubleValueInProperty(separationVisualization.GetProperty("Height"), 2.0);
            SetDoubleValueInProperty(separationVisualization.GetProperty("Sections"), 36.0);

            IMaterialProperty materialProperty = (IMaterialProperty) separationVisualization.GetProperty("Material");
            materialProperty.Value = app.FindMaterial("transp_red", false);

            IExpressionProperty expressionProperty = (IExpressionProperty) separationVisualization.GetProperty("Radius");
            expressionProperty.Expression = "SeparationDistance";

            ICylinderFeature speedMonitoredArea =  separationVisualization.CreateFeature<ICylinderFeature>();
            speedMonitoredArea.SetName("speedMonitoredArea", true);
            
            SetDoubleValueInProperty(speedMonitoredArea.GetProperty("Height"), 1.0);
            SetDoubleValueInProperty(speedMonitoredArea.GetProperty("Sections"), 36.0);

            IMaterialProperty materialPropertyCool = (IMaterialProperty) speedMonitoredArea.GetProperty("Material");
            materialPropertyCool.Value = app.FindMaterial("transp_yellow", false);

            IExpressionProperty expressionPropertyCool = (IExpressionProperty) speedMonitoredArea.GetProperty("Radius");
            expressionPropertyCool.Value = "SeparationDistance*CooldownFactor";
        }

        /// <summary>
        /// Simple function to update the size of the cylinder which visualizes the current separation distance.
        /// </summary>
        private void UpdateVisualization()
        {
            ISimComponent comp = app.World.FindComponent("SeparationVisualization_" + component.Name);
            if (useSSM && comp != null)
            {
                ISimNode node = component.FindNode("mountplate");

                Matrix matrix = comp.TransformationInReference;
                matrix.SetP(new Vector3(node.TransformationInWorld.Px, node.TransformationInWorld.Py, 202));

                comp.TransformationInReference = matrix;

                IFeature cylinder = comp.FindFeature("SeparationVisualization");
                SetDoubleValueInProperty(comp.GetProperty("SeparationDistance"), separationDistance);

                cylinder.Rebuild();
            }
        }

        private void SetDoubleValueInProperty(IProperty prop, double value)
        {
            if (prop is IExpressionProperty propExpr)
            {
                propExpr.Value = value + " {mm}";
            }
            else if (prop is IDoubleProperty doubleProp)
            {
                doubleProp.Value = value;
            }
        }
    }
}