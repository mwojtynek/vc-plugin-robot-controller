using System;

using SpeedAndSeparationMonitoring;

using Caliburn.Micro;
using VisualComponents.Create3D;
using Rosi.Components.Sensors;

using RosiTools.Debugger;

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
        SeparationCalculator seperationCalc;

        // Speed Values
        private double m_robot = 30;
        private double m_payload = 0;
        SpeedCalculator speedCalc;

        private bool initializedSSM = false;

        public double MaxSpeed
        {
            get
            {
                return maximumSpeed;
            }
            set
            {
                /* MaxSpeed kann nicht verändert werden, wenn SSM genutzt werden soll.
                 * Sinnvollerweise ändert die SSM dann nämlich die Geschwindigkeit */
                if (!useSSM)
                {
                    if (value < 0)
                    {
                        maximumSpeed = 0;
                    }
                    else
                    {
                        maximumSpeed = value;
                    }
                }
            }
        }

        public void initSSM() {
            seperationCalc = new SeparationCalculator(t_reaction, t_robot_stop, d_intrusion, s_human, s_robot);
            speedCalc = new SpeedCalculator(m_robot, m_payload);
            initializedSSM = true;
        }

        public void updateSSM(LaserScannerHumanDetectedEventArgs data, double contactArea = 1, BodyPart bodypart = BodyPart.Chest)
        {
             // Die bindings sind scheinbar zwar vorhanden, sodass das hier aufgerufen wird. Wird aber nicht genutzt, daher ignorieren.
            if (!useSSM)
            {
                return;
            }
            // Falls die SSM nicht ignoriert wird, muss sie dennoch initialisiert werden...
            if (!initializedSSM)
            {
                IoC.Get<IMessageService>().AppendMessage("SSM not initialized!", MessageLevel.Error);
                return;
            }

            double d = seperationCalc.GetSeparationDistance(data.MoveSpeed, lastSpeed);
            if (IoC.Get<IDebugCall>().CheckValue[0])
            {
                IoC.Get<IMessageService>().AppendMessage(d.ToString() + " " + data.HumanPosition.Length.ToString() + " <-", MessageLevel.Warning);
            }

            if (d > data.HumanPosition.Length) {
                maximumSpeed = 0;
                return;
            }
            maximumSpeed = speedCalc.GetAllowedVelocity(bodypart, data.MoveSpeed, contactArea);
            if (IoC.Get<IDebugCall>().CheckValue[0])
            {
                IoC.Get<IMessageService>().AppendMessage(maximumSpeed.ToString() + " <-", MessageLevel.Warning);
            }
            
        }
    }
}
