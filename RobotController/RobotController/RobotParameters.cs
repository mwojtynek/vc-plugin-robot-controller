﻿using SpeedAndSeparationMonitoring;
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
        public double allowedCartesianSpeed = 0.0;
        public double maxCartesianSpeed = 0.0;
        public SpeedCalculator speedCalculator = null;
        public SeparationCalculator seperationCalculator = null;
        public double currentMotionStartTime = 0.0;
        public double currentMotionEndTime = 0.0;
        public Matrix lastTcpWorldPosition = Matrix.Zero;

    }
}