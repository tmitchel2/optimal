/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

#pragma warning disable RS1035 // Do not use APIs banned for analyzers
using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;

namespace Optimal.Analyzers
{
    public static class GeneratorLogging
    {
        private static readonly List<string> s_logMessages = [];
        private static string? s_logFilePath;
        private static readonly object s_lock = new();
        private static LoggingLevel s_loggingLevel = LoggingLevel.Info;

        public static void SetLoggingLevel(LoggingLevel level)
        {
            s_loggingLevel = level;
        }

        public static void SetLogFilePath(string path)
        {
            s_logFilePath = path;
        }

        public static LoggingLevel GetLoggingLevel()
        {
            return s_loggingLevel;
        }

        public static void LogMessage(string message, LoggingLevel messageLogLevel = LoggingLevel.Info)
        {
            lock (s_lock)
            {
                try
                {
                    if (s_logFilePath is null)
                    {
                        return;
                    }
                    if (File.Exists(s_logFilePath) is false)
                    {
                        File.WriteAllText(s_logFilePath, string.Empty);
                    }
                    if (messageLogLevel < s_loggingLevel)
                    {
                        return;
                    }
                    string _logMessage = message + "\n";
                    if (messageLogLevel > LoggingLevel.Info)
                    {
                        _logMessage = $"[{messageLogLevel} start]\n" + _logMessage + $"[{messageLogLevel} end]\n\n";
                    }
                    if (!s_logMessages.Contains(_logMessage))
                    {
                        File.AppendAllText(s_logFilePath, _logMessage);
                        s_logMessages.Add(_logMessage);
                    }
                }
                catch (Exception ex)
                {
                    if (s_logFilePath is null)
                    {
                        return;
                    }
                    File.AppendAllText(s_logFilePath, $"[-] Exception occurred in logging: {ex.Message} \n");
                }
            }
        }

        public static void EndLogging()
        {
            if (s_logFilePath is null)
            {
                return;
            }
            if (File.Exists(s_logFilePath))
            {
                File.AppendAllText(s_logFilePath, $"[+] Logging ended at {GetDateTimeUtc()}\n");
            }
        }

        public static string GetDateTimeUtc()
        {
            return DateTime.UtcNow.ToString("yyyy-MM-dd HH:mm:ss.fff", CultureInfo.InvariantCulture);
        }
    }
}
