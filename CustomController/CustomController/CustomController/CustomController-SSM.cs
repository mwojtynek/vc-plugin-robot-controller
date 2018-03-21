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

        // Speed Values
        private double m_robot = 30;
        private double m_payload = 0;
        SpeedCalculator speedCalculator;

        public void InitSSM(Object sender, EventArgs data)
        {
            separationCalculator = new SeparationCalculator(t_reaction, t_robot_stop, d_intrusion, s_human, s_robot);
            speedCalculator = new SpeedCalculator(m_robot, m_payload);

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
                double separationDistance = separationCalculator.GetSeparationDistance(data.MoveSpeed, speedTowardsHuman);
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
            //Printer.print("Speed towards Human: " + speedTowardsHuman.ToString());

            return speedTowardsHuman;
        }
    }
}
