using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using VisualComponents.Create3D;

namespace CustomController
{
    public partial class CustomController
    {
        private void CreateStatisticsComponent()
        {
            if (FindStatisticsComponent() == null)
            {
                ISimComponent statisticsComponent = app.World.CreateComponent("Statistics_" + component.Name);
                statisticsComponent.IsVisible = false;

                SetStatisticValue(statisticsComponent, "SeparationDistance", 0.0);
                SetStatisticValue(statisticsComponent, "SpeedMonitoredArea", 0.0);
                SetStatisticValue(statisticsComponent, "HumanDistance", 0.0);
                SetStatisticValue(statisticsComponent, "HumanAngle", 0.0);
                SetStatisticValue(statisticsComponent, "TcpSpeed", 0.0);
                SetStatisticValue(statisticsComponent, "AllowedSpeed", 0.0);
            }
        }

        private void UpdateStatisticsComponent()
        {
            ISimComponent statisticsComponent = FindStatisticsComponent();
            if (statisticsComponent != null)
            {
                SetStatisticValue(statisticsComponent, "SeparationDistance", separationDistance);
                SetStatisticValue(statisticsComponent, "SpeedMonitoredArea", separationDistance * cooldownFactor);
                SetStatisticValue(statisticsComponent, "HumanDistance", humanDistance);
                SetStatisticValue(statisticsComponent, "HumanAngle", humanAngle);
                SetStatisticValue(statisticsComponent, "TcpSpeed", tcpSpeed.Norm);
                SetStatisticValue(statisticsComponent, "AllowedSpeed", allowedSpeed);
            }
        }

        private ISimComponent FindStatisticsComponent()
        {
            string name = "Statistics_" + component.Name;
            return app.World.FindComponent(name);
        }

        /// <summary>
        /// Sets the desired property in a component to a specifed value. Creates the property if it does not exist.
        /// </summary>
        /// <param name="statisticsComponent"></param>
        /// <param name="statisticName"></param>
        /// <param name="value"></param>
        private void SetStatisticValue(ISimComponent statisticsComponent, string statisticName, double value)
        {
            IDoubleProperty doubleProperty = (IDoubleProperty) statisticsComponent.GetProperty(statisticName);
            if(doubleProperty == null)
            {
                doubleProperty = (IDoubleProperty) statisticsComponent.CreateProperty(typeof(double), PropertyConstraintType.NotSpecified, statisticName);
            }

            doubleProperty.Value = value;
        }
    }
}
