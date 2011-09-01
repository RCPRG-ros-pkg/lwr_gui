# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Willow Garage nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import roslib
roslib.load_manifest('lwr_dashboard')

import wx
import wx.aui
import wx.py.shell
import rxtools
import rxtools.cppwidgets as rxtools

import robot_monitor
from robot_monitor.robot_monitor_panel import RobotMonitorPanel
import diagnostic_msgs.msg
import std_msgs.msg
import std_srvs.srv

import rospy
from roslib import rosenv

from os import path
import threading

from status_control import StatusControl
from fri_control import FRIControl
from robot_control import RobotControl
from diagnostics_frame import DiagnosticsFrame
from rosout_frame import RosoutFrame

class MainFrame(wx.Frame):
    _CONFIG_WINDOW_X = "/Window/X"
    _CONFIG_WINDOW_Y = "/Window/Y"

    def __init__(self, parent, id=wx.ID_ANY, title='Kuka LWR Dashboard', pos=wx.DefaultPosition, size=(400, 50), style=wx.CAPTION | wx.CLOSE_BOX | wx.STAY_ON_TOP):
        wx.Frame.__init__(self, parent, id, title, pos, size, style)

        wx.InitAllImageHandlers()

        rospy.init_node('lwr_dashboard', anonymous=True)
        try:
            getattr(rxtools, "initRoscpp")
            rxtools.initRoscpp("lwr_dashboard_cpp", anonymous=True)
        except AttributeError:
            pass

        self.SetTitle('Kuka LWR Dashboard (%s)' % rosenv.get_master_uri())

        icons_path = path.join(roslib.packages.get_pkg_dir('lwr_dashboard'), "icons/")


        self._robots = rospy.get_param("robots", [""])
        
        if (len(self._robots) == 0):
            self._has_many = False
        else:
            self._has_many = True


        sizer = wx.BoxSizer(wx.HORIZONTAL)
        self.SetSizer(sizer)

        static_sizer = wx.StaticBoxSizer(wx.StaticBox(self, wx.ID_ANY, " ROS state "), wx.HORIZONTAL)
        sizer.Add(static_sizer, 0)
        
        # Diagnostics
        self._diagnostics_button = StatusControl(self, wx.ID_ANY, icons_path, "btn_diag", True)
        self._diagnostics_button.SetToolTip(wx.ToolTip("Diagnostics"))
        static_sizer.Add(self._diagnostics_button, 0)

        # Rosout
        self._rosout_button = StatusControl(self, wx.ID_ANY, icons_path, "btn_rosout", True)
        self._rosout_button.SetToolTip(wx.ToolTip("Rosout"))
        static_sizer.Add(self._rosout_button, 0)

        self.FRI_COMMAND = 1
        self.FRI_MONITOR = 2

        self._fri_state_button = {}
        self._fri_control = {}
        self._fri_mode_topic = {}
        self._robot_control = {}

        for robot in self._robots:
            robot_sizer = wx.StaticBoxSizer(wx.StaticBox(self, wx.ID_ANY, " %s " % robot), wx.HORIZONTAL)


            # FRI State        
            static_sizer = wx.StaticBoxSizer(wx.StaticBox(self, wx.ID_ANY, " FRI "), wx.HORIZONTAL)
            robot_sizer.Add(static_sizer, 0)
            
            self._fri_state_button[robot] = StatusControl(self, wx.ID_ANY, icons_path, "btn_state", True)
            self._fri_state_button[robot].SetToolTip(wx.ToolTip("FRI state"))
            self._fri_state_button[robot].Bind(wx.EVT_BUTTON, lambda x: self.on_fri_state_clicked(x, robot))
            static_sizer.Add(self._fri_state_button[robot], 0)
            
            self._fri_control[robot] = FRIControl(self, wx.ID_ANY, icons_path)
            self._fri_control[robot].SetToolTip(wx.ToolTip("FRI: Stale"))
            static_sizer.Add(self._fri_control[robot], 1, wx.EXPAND)
            
            self._fri_mode_topic[robot] = rospy.Publisher("%s/fri_set_mode" % robot, std_msgs.msg.Int32)
            
            # Robot State        
            static_sizer = wx.StaticBoxSizer(wx.StaticBox(self, wx.ID_ANY, " Robot "), wx.HORIZONTAL)
            robot_sizer.Add(static_sizer, 0)
            
            
            self._robot_control[robot] = RobotControl(self, wx.ID_ANY, icons_path)
            self._robot_control[robot].SetToolTip(wx.ToolTip("Robot: Stale"))
            static_sizer.Add(self._robot_control[robot], 1, wx.EXPAND)

            sizer.Add(robot_sizer, 0)




        self._config = wx.Config("elektron_dashboard")

        self.Bind(wx.EVT_CLOSE, self.on_close)

        self.Layout()
        self.Fit()

        self._diagnostics_frame = DiagnosticsFrame(self, wx.ID_ANY, "Diagnostics")
        self._diagnostics_frame.Hide()
        self._diagnostics_frame.Center()
        self._diagnostics_button.Bind(wx.EVT_BUTTON, self.on_diagnostics_clicked)

        self._rosout_frame = RosoutFrame(self, wx.ID_ANY, "Rosout")
        self._rosout_frame.Hide()
        self._rosout_frame.Center()
        self._rosout_button.Bind(wx.EVT_BUTTON, self.on_rosout_clicked)

        self.load_config()

        self._timer = wx.Timer(self)
        self.Bind(wx.EVT_TIMER, self.on_timer)
        self._timer.Start(500)

        self._topic = "diagnostic"

        self._subs = []
        for n in self._robots:
            sub = rospy.Subscriber("%s/%s" % (n, self._topic), diagnostic_msgs.msg.DiagnosticArray, lambda x: self.dashboard_callback(x, n))
            self._subs.append(sub)

        self._dashboard_message = None
        self._last_dashboard_message_time = 0.0

    def __del__(self):
        for sub in self._subs:
            sub.unregister()


    def on_timer(self, evt):
      level = self._diagnostics_frame._diagnostics_panel.get_top_level_state()
      if (level == -1 or level == 3):
         if (self._diagnostics_button.set_stale()):
            self._diagnostics_button.SetToolTip(wx.ToolTip("Diagnostics: Stale"))
      elif (level >= 2):
         if (self._diagnostics_button.set_error()):
            self._diagnostics_button.SetToolTip(wx.ToolTip("Diagnostics: Error"))
      elif (level == 1):
         if (self._diagnostics_button.set_warn()):
            self._diagnostics_button.SetToolTip(wx.ToolTip("Diagnostics: Warning"))
      else:
         if (self._diagnostics_button.set_ok()):
            self._diagnostics_button.SetToolTip(wx.ToolTip("Diagnostics: OK"))

      self.update_rosout()

      if (rospy.get_time() - self._last_dashboard_message_time > 5.0):
          for ctrl in self._fri_control.values(): 
              ctrl.set_stale()
              
          for ctrl in self._robot_control.values(): 
              ctrl.set_stale()
              
          for ctrl in self._fri_state_button.values(): 
              ctrl.set_stale()
          

      if (rospy.is_shutdown()):
        self.Close()

    def on_diagnostics_clicked(self, evt):
        self._diagnostics_frame.Show()
        self._diagnostics_frame.Raise()

    def on_rosout_clicked(self, evt):
        self._rosout_frame.Show()
        self._rosout_frame.Raise()

    def dashboard_callback(self, msg, robot):
      wx.CallAfter(self.new_dashboard_message, msg, robot)


    def new_dashboard_message(self, msg, robot):
      self._dashboard_message = msg
      self._last_dashboard_message_time = rospy.get_time()

      fri_status = {}
      robot_status = {}
      
      for status in msg.status:
          print robot
          print status.name
          
          if status.name == "FRI state":
              for value in status.values:
                  fri_status[value.key] = value.value
          if status.name == "robot state":
              for value in status.values:
                  robot_status[value.key] = value.value

 
      if (fri_status):
        self._fri_control[robot].set_state(fri_status)
        self.update_fri(fri_status, robot)
      else:
        self._fri_control[robot].set_stale()
        
      if (robot_status):
        self._robot_control[robot].set_state(robot_status)
      else:
        self._robot_control[robot].set_stale()



    ###################################################################
    # FRI related stuff
    ###################################################################

    def update_fri(self, msg, robot):
        if (msg["State"] == "command"):
            self._fri_state_button[robot].set_ok()
        else:
            self._fri_state_button[robot].set_warn()

    def on_fri_state_clicked(self, ev, robot):
        menu = wx.Menu()
        menu.Bind(wx.EVT_MENU, lambda x: self.on_monitor(x, robot), menu.Append(wx.ID_ANY, "Monitor"))
        menu.Bind(wx.EVT_MENU, lambda x: self.on_command(x, robot), menu.Append(wx.ID_ANY, "Command"))
        
        self.PopupMenu(menu)

    def on_monitor(self, evt, robot):
        self.set_mode(self.FRI_MONITOR, robot)
        
    def on_command(self, evt, robot):
        self.set_mode(self.FRI_COMMAND, robot)
    
    def set_mode(self, mode, robot):
        self._fri_mode_topic[robot].publish(std_msgs.msg.Int32(mode))






    def update_rosout(self):
        summary_dur = 30.0
        #if (rospy.get_time() < 30.0):
        #    summary_dur = rospy.get_time() - 1.0

        #if (summary_dur < 0):
        #    summary_dur = 0.0

        summary = self._rosout_frame.get_panel().getMessageSummary(summary_dur)

        if (summary.fatal or summary.error):
            self._rosout_button.set_error()
        elif (summary.warn):
            self._rosout_button.set_warn()
        else:
            self._rosout_button.set_ok()

        tooltip = ""
        if (summary.fatal):
            tooltip += "\nFatal: %s"%(summary.fatal)
        if (summary.error):
            tooltip += "\nError: %s"%(summary.error)
        if (summary.warn):
            tooltip += "\nWarn: %s"%(summary.warn)
        if (summary.info):
            tooltip += "\nInfo: %s"%(summary.info)
        if (summary.debug):
            tooltip += "\nDebug: %s"%(summary.debug)
        
        if (len(tooltip) == 0):
            tooltip = "Rosout: no recent activity"
        else:
            tooltip = "Rosout: recent activity:" + tooltip
        
        if (tooltip != self._rosout_button.GetToolTip().GetTip()):
            self._rosout_button.SetToolTip(wx.ToolTip(tooltip))



    def load_config(self):
      # Load our window options
      (x, y) = self.GetPositionTuple()
      (width, height) = self.GetSizeTuple()
      if (self._config.HasEntry(self._CONFIG_WINDOW_X)):
          x = self._config.ReadInt(self._CONFIG_WINDOW_X)
      if (self._config.HasEntry(self._CONFIG_WINDOW_Y)):
          y = self._config.ReadInt(self._CONFIG_WINDOW_Y)

      self.SetPosition((x, y))
      self.SetSize((width, height))



    def save_config(self):
      config = self._config

      (x, y) = self.GetPositionTuple()
      (width, height) = self.GetSizeTuple()
      config.WriteInt(self._CONFIG_WINDOW_X, x)
      config.WriteInt(self._CONFIG_WINDOW_Y, y)

      config.Flush()

    def on_close(self, event):
      self.save_config()

      self.Destroy()

