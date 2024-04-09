#!/usr/bin/python
# -*- coding: utf-8 -*-
import time
import sys
import logging
import os
import platform


class Logger:
    _instance = None

    STATE_OFF = 0x00
    STATE_ON = 0x01

    LEVEL_NONE = 0x00
    LEVEL_TRANCE = 0x01
    LEVEL_DEBUG = 0x02
    LEVEL_WARNING = 0x03
    LEVEL_ERROR = 0x04

    def __new__(cls):
        if not cls._instance:
            cls._instance = super().__new__(cls)

            cls._instance.state = 0x01
            cls._instance.level = 0x00

            cls._instance.__init_log()

        return cls._instance

    def __init__(self):
        pass

    # 初始化log文件
    def __init_log(self):

        LOG_FORMAT = "%(asctime)s.%(msecs)03d - %(levelname)s - %(message)s"
        DATE_FORMAT = "%Y/%m/%d %H:%M:%S"
        # INFO DEBUG WARNING ERROR CRITICAL
        LOG_LEVEL = logging.INFO

        project_path = os.path.dirname(os.path.abspath(__file__))  # 获取当前文件路径的上一级目录
        exe_file_path = project_path
        print(exe_file_path)

        sysstr = platform.system()
        if (sysstr == "Windows"):
            self.is_windows_flag = True
        else:
            self.is_windows_flag = False
        if self.is_windows_flag == True:
            path = exe_file_path + "\\log\\"
            name = str("{:02d}\{:02d}\\".format(time.localtime().tm_year, time.localtime().tm_mon))
        else:
            path = './log/'
            name = str("{:02d}/{:02d}/".format(time.localtime().tm_year, time.localtime().tm_mon))
        self.log_dir_path = path + name

        isExists = os.path.exists(self.log_dir_path)
        if not isExists:
            os.makedirs(self.log_dir_path)
            print('Log Created', self.log_dir_path)
        else:
            print('Log Already Existed', self.log_dir_path)
        self.log_name = str(
            "{:02d}_{:02d}_{:02d}_{:02d}.{:02d}.{:02d}".format(time.localtime().tm_year, time.localtime().tm_mon,
                                                               time.localtime().tm_mday, time.localtime().tm_hour,
                                                               time.localtime().tm_min, time.localtime().tm_sec))
        self.log_dir = self.log_dir_path + self.log_name + ".log"

        logging.basicConfig(filename=self.log_dir, level=LOG_LEVEL, format=LOG_FORMAT, datefmt=DATE_FORMAT)
        logging.log(logging.INFO, "\n\n########## The robot is starting !!! ##########\n\n\n")

    def print_log_file_debug(self, str_temp):
        logging.log(logging.DEBUG, str_temp)

    def print_log_file_info(self, str_temp):
        logging.log(logging.INFO, str_temp)

    def print_log_file_warning(self, str_temp):
        logging.log(logging.WARNING, str_temp)

    def print_log_file_error(self, str_temp):
        logging.log(logging.ERROR, str_temp)

    def print_log_file_critical(self, str_temp):
        logging.log(logging.CRITICAL, str_temp)

    # 设置时间戳最小单位
    # unit='us' 'ms' 's'
    def __time(self, unit='us'):
        if unit == 'ms' or unit == 'us':
            t = time.time()
            tmid = int(t * 1000000 % 1000000)
            tms = int(tmid / 1000 % 1000)
            t_get = time.localtime(t)
            if unit == 'us':
                tus = int(tmid % 1000)
                t_usr = [t_get.tm_year, t_get.tm_mon, t_get.tm_mday, t_get.tm_hour, t_get.tm_min, t_get.tm_sec, tms,
                         tus]
                t_usr_time_str = str(t_usr[0]).rjust(4) + '.' + str(t_usr[1]).rjust(2, '0') + '.' + str(t_usr[2]).rjust(
                    2, '0') + ' ' + str(t_usr[3]).rjust(2, '0') + ':' + str(t_usr[4]).rjust(2, '0') + ':' + str(
                    t_usr[5]).rjust(2, '0') + '.' + str(t_usr[6]).rjust(3, '0') + '.' + str(t_usr[7]).rjust(3, '0')
            else:
                t_usr = [t_get.tm_year, t_get.tm_mon, t_get.tm_mday, t_get.tm_hour, t_get.tm_min, t_get.tm_sec, tms]
                t_usr_time_str = str(t_usr[0]).rjust(4) + '.' + str(t_usr[1]).rjust(2, '0') + '.' + str(t_usr[2]).rjust(
                    2, '0') + ' ' + str(t_usr[3]).rjust(2, '0') + ':' + str(t_usr[4]).rjust(2, '0') + ':' + str(
                    t_usr[5]).rjust(2, '0') + '.' + str(t_usr[6]).rjust(3, '0')
            return '[' + t_usr_time_str + ']'
        else:
            now = '[' + time.strftime("%Y-%m-%d %H:%M:%S") + ']'
        return now

    def print(self, *objects, sep=' ', end='\n', file=sys.stdout, flush=False):
        if self.state == Logger.STATE_ON:
            print(*objects, sep=sep, end=end, file=file, flush=flush)

    def print_line(self, *objects, sep=' ', end='\n', file=sys.stdout, flush=False):
        if self.state == Logger.STATE_ON:
            print(*objects, sep=sep, end=end, file=file, flush=flush)

    # 设置控制台显示颜色
    # 显示方式: 0（默认值）、1（高亮）、22（非粗体）、4（下划线）、24（非下划线）、 5（闪烁）、25（非闪烁）、7（反显）、27（非反显）
    # 前景色: 30（黑色）、31（红色）、32（绿色）、 33（黄色）、34（蓝色）、35（洋 红）、36（青色）、37（白色）
    # 背景色: 40（黑色）、41（红色）、42（绿色）、 43（黄色）、44（蓝色）、45（洋 红）、46（青色）、47（白色）
    # 常见开头格式：
    # \033[0m            默认字体正常显示，不高亮
    # \033[0;31m       红色字体正常显示
    # \033[1;32;40m  显示方式: 高亮    字体前景色：绿色  背景色：黑色
    # \033[0;31;46m  显示方式: 正常    字体前景色：红色  背景色：青色
    def print_trace(self, *objects, sep=' ', end='\n', file=sys.stdout, flush=False):
        if self.state == Logger.STATE_ON:
            if self.level <= Logger.LEVEL_TRANCE:
                now = self.__time()
                print(now, "\033[0m Info:   \033[0m", end=' ')
                print(*objects, sep=sep, end=end, file=file, flush=flush)

    def print_trace_debug(self, *objects, sep=' ', end='\n', file=sys.stdout, flush=False):
        if self.state == Logger.STATE_ON:
            if self.level <= Logger.LEVEL_DEBUG:
                now = self.__time()
                print(now, "\033[0;31m Debug:  \033[0m", end=' ')
                print(*objects, sep=sep, end=end, file=file, flush=flush)

    def print_trace_warning(self, *objects, sep=' ', end='\n', file=sys.stdout, flush=False):
        if self.state == Logger.STATE_ON:
            if self.level <= Logger.LEVEL_WARNING:
                now = self.__time()
                print(now, "\033[0;34;43m Warning:\033[0m", end=' ')
                print(*objects, sep=sep, end=end, file=file, flush=flush)

    def print_trace_error(self, *objects, sep=' ', end='\n', file=sys.stdout, flush=False):
        if self.state == Logger.STATE_ON:
            if self.level <= Logger.LEVEL_ERROR:
                now = self.__time()
                print(now, "\033[0;32;41m Error:  \033[0m", end=' ')
                print(*objects, sep=sep, end=end, file=file, flush=flush)

    def print_file(self, *objects, sep=' ', end='\n', file=sys.stdout, flush=False):
        if self.state == Logger.STATE_ON:
            print(*objects, sep=sep, end=end, file=file, flush=flush)

    def print_file_trace(self, *objects, sep=' ', end='\n', file=sys.stdout, flush=False):
        if self.state == Logger.STATE_ON:
            print(*objects, sep=sep, end=end, file=file, flush=flush)
