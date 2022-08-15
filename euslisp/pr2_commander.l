#!/usr/bin/env roseus
(load "package://mohou_ros/euslisp/common.l")
(load "package://pr2eus/pr2-interface.l")

(pr2-init)
(ros::subscribe "/mohou_control_command"
  mohou_ros::ControlCommand
  #'(lambda (msg) (send-command *pr2* *ri* msg)))
(ros::spin)
