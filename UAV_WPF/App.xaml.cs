using log4net;
using System;
using System.Collections.Generic;
using System.Configuration;
using System.Data;
using System.Linq;
using System.Reflection;
using System.Threading.Tasks;
using System.Windows;
using UAV_WPF.Tools;

namespace UAV_WPF
{
    /// <summary>
    /// App.xaml 的交互逻辑
    /// </summary>
    public partial class App : Application
    {
        public static readonly ILog log = LogManager.GetLogger(System.Reflection.MethodBase.GetCurrentMethod().DeclaringType);
        protected override void OnStartup(StartupEventArgs e)
        {
           
          //  log4net.Config.XmlConfigurator.Configure();
            base.OnStartup(e);
            //Type type = MethodBase.GetCurrentMethod().DeclaringType;
            //Log4netHelper.WriteLogger(type,LogLevel.Info, " == Startup =====================>>> ");
            //Log4netHelper.WriteLogger(type, LogLevel.Debug, "测试记录Debug日志");
            //Log4netHelper.WriteLogger(type, LogLevel.Error, "测试记录Error日志");               
        }
        protected override void OnExit(ExitEventArgs e)
        {
           
            
        }
    }
}
