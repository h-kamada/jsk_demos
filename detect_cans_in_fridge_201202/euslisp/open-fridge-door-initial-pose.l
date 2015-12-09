
(ros::load-ros-manifest "roseus_smach")
(ros::roseus "open_fridge-door_initial_pose_server")

(defvar *dryrun* (ros::get-param "/dryrun"))
(when (null *dryrun*)
  (load "package://pr2eus/pr2-interface.l")
  (load "package://jsk_demo_common/euslisp/pr2-action.l")
  (pr2-init)
  )

(defun cb (server goal)
  (ros::ros-info "open-fridge-door-initial-pose callback func called")
  (cond
   (*dryrun*
    )
   (t
    (open-fridge-door-initial-pose)
    (let (msg)
      (setq msg (instance std_msgs::Bool))
      (send msg :data t)
      (ros::publish "detectflag" msg))
    ))
  (send server :set-succeeded (send server :result))
  )

(setq s (instance ros::simple-action-server :init
                  "open_fridge_door_initial_pose" roseus_smach::Sub5Action
                  :execute-cb 'cb))

(ros::advertise "detectflag" std_msgs::Bool 10)

(ros::rate 10)
(do-until-key
 (send s :worker)
  (ros::spin-once)
  (ros::sleep))

(exit)

