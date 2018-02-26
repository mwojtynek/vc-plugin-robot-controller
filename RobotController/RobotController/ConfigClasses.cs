using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using System.Configuration;
using Caliburn.Micro;
using System.IO;
using VisualComponents.Create3D;

using RosiTools.Printer;

namespace RobotController
{

    public class ConfigReader {
        public static Configuration config;

        public static void init() {
            String configFile = Path.Combine(IoC.Get<IApplicationSettingService>().UserConfigFolder, "RobotController.config");
            ExeConfigurationFileMap map = new ExeConfigurationFileMap();
            map.ExeConfigFilename = configFile;
            config = ConfigurationManager.OpenMappedExeConfiguration(map, ConfigurationUserLevel.None);
        }

        public static RobotSection readSection(String name) {
            RobotSection sec = config.Sections[name] as RobotSection;
            
            if (sec == null) {
                sec = new RobotSection();
                sec.obsFile.Path = "C:\\defaultPath";
                sec.urdfFile.Path = "C:\\defaultPath2";
                config.Sections.Add(name, sec);
                config.Save();
            }
            return sec;
        }

        internal static StapleSection readStapleConfig()
        {
            StapleSection sec = config.Sections["staplePath"] as StapleSection;

            if (sec == null)
            {
                sec = new StapleSection();
                sec.staplePath.Path = "C:\\local\\autoLabel\\itsowl-tt-autolabel\\autoLabel-Sim-Models\\stl-files\\";
                config.Sections.Add("staplePath", sec);
                config.Save();
            }
            return sec;
        }
    }

    public class StapleSection: ConfigurationSection
    {
        [ConfigurationProperty("staplePath", IsDefaultCollection = true)]
        public StaplePath staplePath
        {
            get { return (StaplePath)base["staplePath"]; }
            set { base["staplePath"] = value; }
        }
    }

    public class RobotSection: ConfigurationSection
    {
               
        [ConfigurationProperty("UrdfFile",
           IsDefaultCollection = true)]
        public UrdfFile urdfFile
        {
            get { return (UrdfFile)base["UrdfFile"]; }
            set { base["UrdfFile"] = value; }
        }
        
        [ConfigurationProperty("ObstacleFile",
           IsDefaultCollection = true)]
        public ObstacleFile obsFile
        {
            get { return (ObstacleFile)base["ObstacleFile"]; }
            set { base["ObstacleFile"] = value; }
        }
        
    }


    public sealed class StaplePath : ConfigurationElement
    {
        [ConfigurationProperty("Path",
            DefaultValue = "C:\\local\\autoLabel\\itsowl-tt-autolabel\\autoLabel-Sim-Models\\stl-files\\",
           IsDefaultCollection = true)]
        public String Path
        {
            get { return (string)base["Path"]; }
            set { base["Path"] = value; }
        }
    }

    public sealed class UrdfFile : ConfigurationElement
    {
        [ConfigurationProperty("Path",
            DefaultValue = "C:\\Users\\fresh\\Documents\\workspaceAutolabel\\rosi.plugin.pathplanner\\robot_descriptions\\urdf\\lbr_iiwa_14_r820-MUKGripper.urdf",
           IsDefaultCollection = true)]
        public String Path
        {
            get { return (string)base["Path"]; }
            set { base["Path"] = value; }
        }
    }


    public sealed class ObstacleFile : ConfigurationElement
    {
        private String _path;

        [ConfigurationProperty("Path",
            DefaultValue = "C:\\Users\\fresh\\Documents\\workspaceAutolabel\\rosi.plugin.pathplanner\\cage-models\\tablesBothiiwa-scaled.stl",
           IsDefaultCollection = true)]
        public String Path
        {
            get { return (string)base["Path"]; }
            set { base["Path"] = value; }
        }
    }
}