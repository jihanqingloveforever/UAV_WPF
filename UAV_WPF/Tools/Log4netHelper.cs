using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace UAV_WPF.Tools
{
    class Log4netHelper
    {
        public static void WriteLogger(Type type, LogLevel logLevel, string msg)
        {
            log4net.ILog log = log4net.LogManager.GetLogger(type);
            switch (logLevel)
            {
                case LogLevel.Debug: log.Debug(msg); break;
                case LogLevel.Error: log.Error(msg); break;
                case LogLevel.Fatal: log.Fatal(msg); break;
                case LogLevel.Info: log.Info(msg); break;
                case LogLevel.Warn: log.Warn(msg); break;
                default: break;
            }
        }
    }
    /// <summary>
    /// 日志记录级别
    /// </summary>
    public enum LogLevel
    {
        Debug,
        Info,
        Warn,
        Error,
        Fatal
    }
}
