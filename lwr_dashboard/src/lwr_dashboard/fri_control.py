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

class FRIControl(wx.Window):
  def __init__(self, parent, id, icons_path):
    wx.Window.__init__(self, parent, id, wx.DefaultPosition, wx.Size(60, 50))

    bitmap = wx.Bitmap(path.join(icons_path, "state.png"), wx.BITMAP_TYPE_PNG)
    self._state_bitmap = (bitmap.GetSubBitmap(wx.Rect(  0, 0, 40, 40)),
                          bitmap.GetSubBitmap(wx.Rect( 40, 0, 40, 40)))

    self._state = "monitor"
    self._quality = "PERFECT"
    self._latency = "0"

    self.SetSize(wx.Size(160, 50))

    self.Bind(wx.EVT_PAINT, self.on_paint)

  def on_paint(self, evt):
    dc = wx.BufferedPaintDC(self)

    dc.SetBackground(wx.Brush(self.GetBackgroundColour()))
    dc.Clear()

    w = self.GetSize().GetWidth()
    h = self.GetSize().GetHeight()

    if (self._state == "command"):
        dc.DrawBitmap(self._state_bitmap[0], 0, 0, True)
    else:
        dc.DrawBitmap(self._state_bitmap[1], 0, 0, True)

    fnt = dc.GetFont()
    fnt.SetPointSize(7)
    dc.SetFont(fnt)
    dc.DrawText("Quality: %s" % self._quality, 40, 0)
    dc.DrawText("Latency: %s" % self._latency, 40, 12)


  def set_state(self, msg):
    self._state = msg["State"]
    self._quality = msg["Quality"]
    self._latency = msg["Latency"]
    self.SetToolTip(wx.ToolTip("Tooltip to be set some day..."))
    
    self.Refresh()

  def set_stale(self):
    self.SetToolTip(wx.ToolTip("FRI: Stale"))

    self.Refresh()
