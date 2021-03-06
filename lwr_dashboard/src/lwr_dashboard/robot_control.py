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

from os import path

class RobotControl(wx.Window):
  def __init__(self, parent, id, icons_path):
    wx.Window.__init__(self, parent, id, wx.DefaultPosition, wx.Size(60, 50))

    bitmap = wx.Bitmap(path.join(icons_path, "btn_motors.png"), wx.BITMAP_TYPE_PNG)
    self._state_bitmap = (bitmap.GetSubBitmap(wx.Rect(0,   0, 40, 40)),
                          bitmap.GetSubBitmap(wx.Rect(40,  0, 40, 40)),
                          bitmap.GetSubBitmap(wx.Rect(80,  0, 40, 40)),
                          bitmap.GetSubBitmap(wx.Rect(120, 0, 40, 40)))

    bitmap = wx.Bitmap(path.join(icons_path, "long_buttons.png"), wx.BITMAP_TYPE_PNG)
    self._screen_bitmap = (bitmap.GetSubBitmap(wx.Rect(0, 0, 80, 40)),
                          bitmap.GetSubBitmap(wx.Rect( 80, 40, 80, 40)),
                          bitmap.GetSubBitmap(wx.Rect( 0,  40, 80, 40)))

    self._status = {}
    self._stale = True

    self.SetSize(wx.Size(120, 40))

    self.Bind(wx.EVT_PAINT, self.on_paint)

  def on_paint(self, evt):
    dc = wx.BufferedPaintDC(self)

    dc.SetBackground(wx.Brush(self.GetBackgroundColour()))
    dc.Clear()

#    w = self.GetSize().GetWidth()
#    h = self.GetSize().GetHeight()
#
#    qual = "?"
#    rate = "?"
#

    strategy = "?"

    allok = False
    
    if (self._stale):
        dc.DrawBitmap(self._state_bitmap[3], 0, 0, True)
        dc.DrawBitmap(self._screen_bitmap[2], 40, 0, True)
    else:
        if (self._status["Error"] == "0000000" and self._status["Warning"] == "0000000"):
            allok = True
        else:
            allok = False
        
        if (allok and self._status["Power"] == "1111111"):
            dc.DrawBitmap(self._state_bitmap[0], 0, 0, True)
        
        if (allok and self._status["Power"] != "1111111"):
            dc.DrawBitmap(self._state_bitmap[1], 0, 0, True)
        
        if (not allok):
            dc.DrawBitmap(self._state_bitmap[2], 0, 0, True)
            
        if (self._status["Control Strategy"] == "Position"):
            strategy = "Pos"
        if (self._status["Control Strategy"] == "Joint impedance"):
            strategy = "J-imp"
        if (self._status["Control Strategy"] == "Cartesian impedance"):
            strategy = "C-imp"
        if (self._status["Control Strategy"] == "Invalid"):
            strategy = "Inv"

        if (strategy != "Inv"):
            dc.DrawBitmap(self._screen_bitmap[0], 40, 0, True)
        else:
            dc.DrawBitmap(self._screen_bitmap[1], 40, 0, True)

    fnt = dc.GetFont()
    fnt.SetPointSize(7)
    dc.SetFont(fnt)
    dc.DrawLabel("Control", wx.Rect(45, 5, 70, 32), wx.ALIGN_LEFT|wx.ALIGN_TOP)
    
    fnt.SetPointSize(14)
    fnt.SetWeight(wx.FONTWEIGHT_BOLD)
    dc.SetFont(fnt)
    dc.DrawLabel("%s" % strategy, wx.Rect(45, 5, 70, 32), wx.ALIGN_RIGHT|wx.ALIGN_BOTTOM)

  def set_state(self, msg):
    self._stale = False
    self._status = msg;
    
    tooltip = ""
    
    for key, value in msg.items():
        tooltip = tooltip + "%s: %s\n" % (key, value)
    
    self.SetToolTip(wx.ToolTip(tooltip))
    
    self.Refresh()

  def set_stale(self):
    self.SetToolTip(wx.ToolTip("Robot: Stale"))
    self._stale = True
    self.Refresh()

