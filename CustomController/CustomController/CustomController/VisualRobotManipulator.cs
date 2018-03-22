using System;
using System.Collections.Generic;
using System.ComponentModel.Composition;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using VisualComponents.Create3D;
using VisualComponents.UX.Shared;
using Caliburn.Micro;
using System.ComponentModel;
using RosiTools.Debugger;
using RosiTools.Printer;
using System.Diagnostics;
using System.Threading;

namespace CustomController
{
    
    public class VisualRobotManipulator {
        public ISimComponent component;
        

        public VisualRobotManipulator(ISimComponent component)
        {
            this.component = component;
            
        }
        
        
        public int jointCount {
            get
            {
                
                int count = this.robot.RobotController.Joints.Count;
                
                return count;
            }
        }

        public IRobot robot {
            get
            {
                
                IRobot robot = component.GetRobot();
                if (robot == null) {
                
                    throw new ArgumentException(component.Name + " is not a robot");
                }
                
                return component.GetRobot();
            }
        }

        public double[] getConfigurationDouble() {

            
            double[] jointVals = new double[jointCount];
            IList<IJoint> joints = this.robot.RobotController.Joints;
            for (int i = 0; i < jointVals.Length; i++) {
                jointVals[i] = joints[i].Value;
            }
            
            return jointVals;
        }

        public Vector getConfiguration(){

            
            Vector ret = new Vector(getConfigurationDouble()); 
            
            return ret;
        }

        public void setConfiguration(double[] joints)
        {

            
            this.robot.RobotController.InvalidateKinChains();
            this.robot.RobotController.SetJointValues(joints);
            

        }

        public void setConfiguration(Vector joints) {

            
            setConfiguration(joints.Elements);
            
        }

    }

}
