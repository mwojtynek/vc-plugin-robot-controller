using System;
using System.ComponentModel.Composition;
using VisualComponents.Create3D;
using VisualComponents.UX.Shared;
using Caliburn.Micro;

namespace CustomController
{
    [Export(typeof(ISimulationTicker))]
    [Export(typeof(IPlugin))]
    public class SimulationTicker : IPlugin, ISimulationTicker
    {

        ISamplingTimer st;
        IApplication app;
        
        private readonly double SAMPLE_TIME = 0.01;


        public double tickTime
        {
            get
            {
                return SAMPLE_TIME;
            }
        }

        private System.Action _timerTick;
        public System.Action timerTick
        {
            get
            {
                return _timerTick;
            }

            set
            {
                _timerTick = value;
            }
        }

        private System.Action _timerStarted;
        public System.Action timerStarted
        {
            get
            {
                return _timerStarted;
            }

            set
            {
                _timerStarted = value;
            }
        }

        private System.Action _timerStopped;
        public System.Action timerStopped
        {
            get
            {
                return _timerStopped;
            }

            set
            {
                _timerStopped = value;
            }
        }

        [ImportingConstructor]
        public SimulationTicker([Import(typeof(IApplication))] IApplication _app)
        {
            app = _app;
        }

        public void Exit()
        {
        }

        public void Initialize()
        {
            st = app.StatisticsManager.CreateTimer(handler);
            st.SamplingInterval = SAMPLE_TIME;

            app.Simulation.SimulationStarted += started;
            app.Simulation.SimulationStopped += stopped;
        }

        private void handler(object sender, EventArgs e)
        {
            timerTick?.Invoke();
        }

        private void stopped(object sender, EventArgs e)
        {
            st.StartStopTimer(false);
            timerStarted?.Invoke();

        }

        private void started(object sender, EventArgs e)
        {
            st.StartStopTimer(true);
            timerStopped?.Invoke();
        }
    }

    public interface ISimulationTicker
    {
        double tickTime { get; }
        System.Action timerTick { get; set; }  
        System.Action timerStarted { get; set; }
        System.Action timerStopped { get; set; }

    }
}
