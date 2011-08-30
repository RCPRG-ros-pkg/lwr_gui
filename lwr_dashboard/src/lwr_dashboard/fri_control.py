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


# 
# 1 - monitor
# 2 - command

import roslib
roslib.load_manifest('lwr_dashboard')
import rospy

import wx

from os import path
import std_msgs

class FRIControl(wx.Window):
  def __init__(self, parent, id, icons_path):
    wx.Window.__init__(self, parent, id, wx.DefaultPosition, wx.Size(60, 40))

    bitmap = wx.Bitmap(path.join(icons_path, "state.png"), wx.BITMAP_TYPE_PNG)
    self._state_bitmap = (bitmap.GetSubBitmap(wx.Rect(40, 0, 40, 40)),
                          bitmap.GetSubBitmap(wx.Rect(0,  0, 40, 40)),
                          bitmap.GetSubBitmap(wx.Rect(80, 0, 40, 40)))

    bitmap = wx.Bitmap(path.join(icons_path, "communication.png"), wx.BITMAP_TYPE_PNG)
    self._qual_bitmap = (bitmap.GetSubBitmap(wx.Rect(40, 0, 40, 40)),
                          bitmap.GetSubBitmap(wx.Rect( 0, 0, 40, 40)),
                          bitmap.GetSubBitmap(wx.Rect(80, 0, 40, 40)),
                          bitmap.GetSubBitmap(wx.Rect(120, 0, 40, 40)))
    
    bitmap = wx.Bitmap(path.join(icons_path, "long_buttons.png"), wx.BITMAP_TYPE_PNG)
    self._screen_bitmap = (bitmap.GetSubBitmap(wx.Rect(0,   0, 80, 40)),
                          bitmap.GetSubBitmap(wx.Rect( 80,  0, 80, 40)),
                          bitmap.GetSubBitmap(wx.Rect( 80, 40, 80, 40)),
                          bitmap.GetSubBitmap(wx.Rect( 0,  40, 80, 40)))

    self._status = {}
    self._stale = True
    
    self.MONITOR = 2
    self.COMMAND = 1

    self.SetSize(wx.Size(120, 40))


    self._fri_mode_topic = rospy.Publisher('fri_set_mode', std_msgs.msg.Int32)

    self.Bind(wx.EVT_PAINT, self.on_paint)
    self.Bind(wx.EVT_LEFT_DOWN, self.on_left_down)

  def on_left_down(self, evt):
    menu = wx.Menu()
    menu.Bind(wx.EVT_MENU, self.on_monitor, menu.Append(wx.ID_ANY, "Monitor"))
    menu.Bind(wx.EVT_MENU, self.on_command, menu.Append(wx.ID_ANY, "Command"))
    
    #self.toggle(True)
    self.PopupMenu(menu)
    #self.toggle(False)

  def on_monitor(self, evt):
    self.set_mode(self.MONITOR)
    
  def on_command(self, evt):
    self.set_mode(self.COMMAND)

  def set_mode(self, mode):
    self._fri_mode_topic.publish(std_msgs.msg.Int32(mode))

  def on_paint(self, evt):
    dc = wx.BufferedPaintDC(self)

    dc.SetBackground(wx.Brush(self.GetBackgroundColour()))
    dc.Clear()

    w = self.GetSize().GetWidth()
    h = self.GetSize().GetHeight()

    qual = "?"
    rate = "?"

    if (self._stale):
        dc.DrawBitmap(self._state_bitmap[2], 0, 0, True)
#        dc.DrawBitmap(self._qual_bitmap[3], 40, 0, True)
        dc.DrawBitmap(self._screen_bitmap[3], 40, 0, True)
    else:
        qual = self._status["Quality"]
        rate = "%.0f" % (1000*float(self._status["Desired Command Sample Time"]))
        
        if (self._status["State"] == "command"):
            dc.DrawBitmap(self._state_bitmap[0], 0, 0, True)
        else:
            dc.DrawBitmap(self._state_bitmap[1], 0, 0, True)
            
        if (qual == "PERFECT" or qual == "OK"):
            dc.DrawBitmap(self._screen_bitmap[0], 40, 0, True)
        if (qual == "BAD"): 
            dc.DrawBitmap(self._screen_bitmap[1], 40, 0, True)
        if (qual == "UNACCEPTABLE"):
            dc.DrawBitmap(self._screen_bitmap[2], 40, 0, True)

    
#        dc.DrawBitmap(self._screen_bitmap[0], 80, 0, True)

    fnt = dc.GetFont()
    fnt.SetPointSize(7)
    dc.SetFont(fnt)
    dc.DrawLabel("Rate [ms]", wx.Rect(45, 5, 70, 32), wx.ALIGN_LEFT|wx.ALIGN_TOP)
    
    fnt.SetPointSize(14)
    fnt.SetWeight(wx.FONTWEIGHT_BOLD)
    dc.SetFont(fnt)
    dc.DrawLabel("%s" % rate, wx.Rect(45, 5, 70, 32), wx.ALIGN_RIGHT|wx.ALIGN_BOTTOM)

  def set_state(self, msg):
    self._stale = False
    self._status = msg;
    tooltip = ""
    
    for key, value in msg.items():
        tooltip = tooltip + "%s: %s\n" % (key, value)
    
    tool = wx.ToolTip(tooltip)
    tool.SetDelay(1)
    self.SetToolTip(tool)
    
    self.Refresh()

  def set_stale(self):
    self.SetToolTip(wx.ToolTip("FRI: Stale"))
    self._stale = True
    self.Refresh()

