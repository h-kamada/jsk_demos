
(ros::load-ros-manifest "roseus_smach")
(ros::roseus "catch_can_server")

(defvar *dryrun* (ros::get-param "/dryrun"))
(when (null *dryrun*)
  (load "package://pr2eus/pr2-interface.l")
  (load "package://jsk_demo_common/euslisp/pr2-action.l")
  (pr2-init)
  )

(defun cb (server goal)
  (ros::ros-info "catch-can callback func called")
  (cond
   (*dryrun*
    (ros::ros-info "move-forward-larm(dryrun): wait 5s")
    (unix:sleep 5)
    (ros::ros-info "move-forward-larm(dryrun): wait end")
    )
   (t
    (catch-can)
    ))
  (let (msg)
    (setq msg (instance std_msgs::Bool))
    (send msg :data nil) ;; finish detecting can
    (ros::publish "detectcanflag" msg))
  (send server :set-succeeded (send server :result))
  (send *pr2* :reset-pose) ;; for another try
  )

(setq s (instance ros::simple-action-server :init
                  "catch_can" roseus_smach::Sub5Action
                  :execute-cb 'cb))

(ros::advertise "detectcanflag" std_msgs::Bool 10)

(ros::rate 10)
(do-until-key
 (send s :worker)
  (ros::spin-once)
  (ros::sleep))

(exit)

