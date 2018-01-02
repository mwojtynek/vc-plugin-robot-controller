
using System;
using System.Timers;
using System.Configuration;

using System.ComponentModel.Composition;
using Caliburn.Micro;
using VisualComponents.Create3D;
using VisualComponents.UX.Shared;
using VisualComponents.UX.Input;
using System.Windows.Input;
using System.IO;
using System.IO.Pipes;
using System.Threading;

namespace Debugger
{


    [Export(typeof(IDebugCall))]
    [Export(typeof(IPlugin))]
    public partial class DebugInterface : IPlugin, IDebugCall
    {

        public System.Action[] DebugCall { get; set; }
        public System.Action[] DebugCheck { get; set; }
        public System.Action[] DebugText { get; set; }
        public System.Action[] DebugNum { get; set; }
        public bool[] CheckValue { get; set; }
        public String[] TextValue { get; set; }
        public double[] NumValue { get; set; }
        public int size { get; set; }


        /////////////////////////////////////////////////////////////////
        
        NamedPipeClientStream pipeClient = null;
        StreamWriter sw = null;
        Thread th = null;

        private void startConnection()
        {
            if (th == null || !th.IsAlive)
            {
                th = new Thread(runThread);
                th.Start();
            }
        }

        public bool push(double value)
        {
            bool server = false;
            if (sw != null)
            {
                try
                {
                    sw.WriteLine(value.ToString());
                    sw.Flush();
                    server = true;
                }
                catch
                {
                    sw = null;
                    startConnection();
                }
            }
            return server;
        }

        public void runThread()
        {
            pipeClient = new NamedPipeClientStream(".", "VCPlotInserter", PipeDirection.Out);
            pipeClient.Connect();
            sw = new StreamWriter(pipeClient);
        }

        /////////////////////////////////////////////////////////////////

        public DebugInterface() {
            size = 3;

            DebugCall = new System.Action[size];
            DebugCheck = new System.Action[size];
            DebugText = new System.Action[size];
            DebugNum = new System.Action[size];
            CheckValue = new bool[size];
            TextValue = new String[size];
            NumValue = new double[size];
        }

        public void Exit()
        {
            
        }

        public void Initialize()
        {
            registerUXSite();
            startConnection();
            IoC.Get<IDebugCall>().DebugCall[0] += pusher;
        }

        private void pusher()
        {
            IoC.Get<IDebugCall>().push(IoC.Get<IDebugCall>().NumValue[0]);
        }
    }
    public interface IDebugCall {
        System.Action[] DebugCall {get; set;}
        System.Action[] DebugCheck {get; set;}
        System.Action[] DebugText { get; set; }
        System.Action[] DebugNum { get; set; }
        bool[] CheckValue { get; set; }
        String[] TextValue { get; set; }
        double[] NumValue { get; set; }
        int size { get; set; }

        bool push(double value);



    }
}
