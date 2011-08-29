#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2011, Robot Control and Pattern Recognition Group, Warsaw University of Technology
#
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the <organization> nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


import roslib
roslib.load_manifest('diag_sim')

import rospy
import diagnostic_msgs.msg


def diag_sim():    
    diag_pub = rospy.Publisher('/diagnostics', diagnostic_msgs.msg.DiagnosticArray)
    rospy.init_node('diag_sim')
    
    cnt = 0
    
    while not rospy.is_shutdown():
        
        cnt = cnt + 1
        
        #Main header
        diag = diagnostic_msgs.msg.DiagnosticArray()
        diag.header.stamp = rospy.Time.now()

        state = ""
        qual = ""
        msg = ""
        warning = ""
        error = ""
        power = ""
        strategy = ""
        lvl = diagnostic_msgs.msg.DiagnosticStatus.OK
        
        #FRI info                                                                                                                              
        stat = diagnostic_msgs.msg.DiagnosticStatus()
        stat.name = "FRI state"
        
    
        [div, mod] = divmod(cnt, 4)
        if mod == 0:
            lvl = diagnostic_msgs.msg.DiagnosticStatus.OK
            msg = "Communication quality PERFECT"
            qual = "PERFECT"
            state = "monitor"
            error = "0000000"
            warning = "0000000"
            power = "1111111"
            strategy = "Position"
        if mod == 1:
            lvl = diagnostic_msgs.msg.DiagnosticStatus.OK
            msg = "Communication quality OK"
            qual = "OK"
            state = "monitor"
            error = "0000000"
            warning = "0000000"
            power = "1110011"
            strategy = "Cartesian impedance"
        if mod == 2:
            lvl = diagnostic_msgs.msg.DiagnosticStatus.WARN
            msg = "Communication quality BAD"
            qual = "BAD"
            state = "command"
            error = "0000001"
            warning = "0000100"
            power = "1111111"
            strategy = "Joint impedance"
        if mod == 3:
            lvl = diagnostic_msgs.msg.DiagnosticStatus.ERROR
            msg = "Communication quality UNACCEPTABLE"
            qual = "UNACCEPTABLE"
            state = "command"
            error = "1000000"
            warning = "0100000"
            power = "1110011"
            strategy = "Invalid"
            
        stat.level = lvl 
        stat.message = msg

        stat.values.append(diagnostic_msgs.msg.KeyValue("Kuka System Time", "0"))
        stat.values.append(diagnostic_msgs.msg.KeyValue("State",            state))
        stat.values.append(diagnostic_msgs.msg.KeyValue("Quality",          qual))
        stat.values.append(diagnostic_msgs.msg.KeyValue("Desired Send Sample Time", "3"))
        stat.values.append(diagnostic_msgs.msg.KeyValue("Desired Command Sample Time", str(mod*0.025+0.025)))
        stat.values.append(diagnostic_msgs.msg.KeyValue("Safety Limits",    "5"))
        stat.values.append(diagnostic_msgs.msg.KeyValue("Answer Rate",      "6"))
        stat.values.append(diagnostic_msgs.msg.KeyValue("Latency",          "7"))
        stat.values.append(diagnostic_msgs.msg.KeyValue("Jitter",           "8"))
        stat.values.append(diagnostic_msgs.msg.KeyValue("Average Missed Answer Packages", "9"))
        stat.values.append(diagnostic_msgs.msg.KeyValue("Total Missed Packages", "10"))

        #append
        diag.status.append(stat)
        
        
        #FRI info                                                                                                                              
        stat = diagnostic_msgs.msg.DiagnosticStatus()
        stat.name = "robot state"
        stat.level = diagnostic_msgs.msg.DiagnosticStatus.OK
        stat.message = "OK"
    

        stat.values.append(diagnostic_msgs.msg.KeyValue("Power",            power))
        stat.values.append(diagnostic_msgs.msg.KeyValue("Control Strategy", strategy))
        stat.values.append(diagnostic_msgs.msg.KeyValue("Error",            error))
        stat.values.append(diagnostic_msgs.msg.KeyValue("Warning",          warning))

        #append
        diag.status.append(stat)
        
        #publish
        diag_pub.publish(diag)
        rospy.sleep(1.0)


if __name__ == '__main__':
    try:
        diag_sim()
    except rospy.ROSInterruptException: pass
    except IOError: pass
    except KeyError: pass
