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
using Debugger;


namespace CustomController
{
    /* Aufgabe 1: Lauschen, was für neue Komponente hinzukommen
        Aus den neuen Komponenten, sollen jene ausgewählt und manipuliert werden, die dazu mit besonderer Behaviour markiert wurden.
       Aufgabe 2: Einen Manipulator zuweisen
     */ 

    class VisualRobotManipulatorCollector
    {

        static readonly string BEHAVIOR_NAME = "CustomController";
        IApplication app;
        Dictionary<string, ISimComponent> robots;
        Dictionary<string, CustomController> controllers;
        public VisualRobotManipulatorCollector(IApplication _app) {
            app = _app;
            robots = new Dictionary<string, ISimComponent>();
            controllers = new Dictionary<string, CustomController>();

            app.World.ComponentAdded += componentAdded;
            app.World.ComponentRemoving += componentRemoved;
            
        }

        private bool isCustomController(ISimComponent component) {
            IBehavior b = component.FindBehavior(BEHAVIOR_NAME);
            if (b != null && b.Type == BehaviorType.Note)
            {
                return true;
            }
            return false;
        }

        private bool isCustomController(IBehavior behavior)
        {
            if (behavior.Type == BehaviorType.Note && behavior.Name == BEHAVIOR_NAME) {
                return true;
            }
            return false;
        }

        private void componentRemoved(object sender, ComponentRemovingEventArgs e)
        {
            if (isCustomController(e.Component)) {
                removeRobot(e.Component.Name);
            }
        }

        // TODO check if Component has the Behavior
        private void componentAdded(object sender, ComponentAddedEventArgs e)
        {
            e.Component.BehaviorAdded += behaviorAdded;
            e.Component.BehaviorRemoving += behaviorRemoved;
            e.Component.GetProperty("Name").PropertyChanged += componentRenamed;
            if (isCustomController(e.Component)) {
                addRobot(e.Component.Name, e.Component);
            }
        }

        private void componentRenamed(object sender, PropertyChangedEventArgs e)
        {
            ISimComponent comp = ((IProperty)sender).Container as ISimComponent;
            if (!isCustomController(comp)) // Wenn es eh kein Controller ist, ist es egal...
            {
                return;
            }

            foreach (String oldNames in robots.Keys) {
                if (app.World.FindComponent(oldNames) == null) {
                    changeRobot(oldNames, comp.Name);
                    break;
                }
            }
        }



        // TODO check if it was a manipulation flag 
        private void behaviorRemoved(object sender, BehaviorRemovingEventArgs e)
        {
            if (isCustomController(e.Behavior)) {
                ISimComponent comp = sender as ISimComponent;
                removeRobot(comp.Name);       
            }
        }

        private void behaviorAdded(object sender, BehaviorAddedEventArgs e)
        {
            if (e.Behavior.Type == BehaviorType.Note) {
                e.Behavior.GetProperty("Name").PropertyChanged += NameChange;
                e.Behavior.GetProperty("Note").PropertyChanged += NoteChange;
            }
            if (isCustomController(e.Behavior)) {
                ISimComponent comp = sender as ISimComponent;
                addRobot(comp.Name, comp);
            }
        }

        private void NoteChange(object sender, PropertyChangedEventArgs e)
        {
            if (isCustomController(((IProperty)sender).Container as IBehavior)) {
                // TODO controller entsprechend manipulieren
            }
        }

        private void NameChange(object sender, PropertyChangedEventArgs e)
        {
            IProperty prop = sender as IProperty;
            ISimComponent comp = ((IBehavior)prop.Container).Node.Component;
            if (prop.Value.ToString() == BEHAVIOR_NAME)
            {
                addRobot(comp.Name, comp);
            }
            else
            {
                removeRobot(comp.Name);
            }
        }

        private void addRobot(string name, ISimComponent comp)
        {
            robots.Add(name, comp);

            controllers.Add(name, new CustomController(app, new VisualRobotManipulator(app, name)));
            IoC.Get<IMessageService>().AppendMessage(name + " hinzugefügt", MessageLevel.Warning);
        }

        private void removeRobot(string name) {
            robots.Remove(name);
            if (controllers.Remove(name))
            {
                IoC.Get<IMessageService>().AppendMessage(name + " entfernt", MessageLevel.Warning);
            }
        }

        private void changeRobot(string oldName, string newName) {
            ISimComponent comp = robots[oldName];
            CustomController manip = controllers[oldName];

            robots.Remove(oldName);
            controllers.Remove(oldName);

            robots.Add(newName, comp);
            controllers.Add(newName, manip);
            
            IoC.Get<IMessageService>().AppendMessage(oldName + " geändert in " + newName, MessageLevel.Warning);
        }
        
    }

    class VisualRobotManipulator {
        private IApplication _app;
        public ISimComponent component;
        public VisualRobotManipulator(IApplication app, string componentName)
        {
            _app = app;
            component = _app.World.FindComponent(componentName);
            if (component == null) {
                throw new ArgumentException();
            }
            if (component.FindFeature("startFrame") == null)
            {
                IFrameFeature start = component.RootNode.RootFeature.CreateFeature<IFrameFeature>();
                start.Name = "startFrame";
            }
            if (component.FindFeature("goalFrame") == null)
            {
                IFrameFeature goal = component.RootNode.RootFeature.CreateFeature<IFrameFeature>();
                goal.Name = "goalFrame";
            }

            //DEBUG FRAME
            if (component.FindFeature("debugFrame") == null)
            {
                IFrameFeature goal = component.RootNode.RootFeature.CreateFeature<IFrameFeature>();
                goal.Name = "debugFrame";
            }

        }
        private bool isRobot(bool throwException) {
            IRobot robot = component.GetRobot();
            if (robot == null)
            {
                if (throwException)
                {
                    throw new ArgumentException();
                }
                else {
                    return false;
                }
            }
            return true;

        }

        public int jointCount {
            get
            {
                isRobot(true);
                return component.GetRobot().RobotController.Joints.Count;
            }
        }

        public IRobot robot {
            get
            {
                return component.GetRobot();
            }
        }

        public double[] getConfiguration() {
            isRobot(true);
            double[] jointVals = new double[jointCount];
            IList<IJoint> joints = component.GetRobot().RobotController.Joints;
            for (int i = 0; i < jointVals.Length; i++) {
                jointVals[i] = joints[i].Value;
            }

            return jointVals;
        }

        public void setConfiguration(double[] joints)
        {
            isRobot(true);
            component.GetRobot().RobotController.SetJointValues(joints);
        }

    }

    // Kann auch in die Collector klasse eingepflegt werden.
    [Export(typeof(IPlugin))]
    class CollectorPlugin : IPlugin
    {
        VisualRobotManipulatorCollector collector;
        [ImportingConstructor]
        public CollectorPlugin([Import(typeof(IApplication))] IApplication app)
        {
            collector = new VisualRobotManipulatorCollector(app);
        }
        public void Exit()
        {
        }

        public void Initialize()
        {
        }
    }
}
