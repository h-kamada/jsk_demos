
(ros::load-ros-manifest "roseus_smach")
(ros::roseus "solve_ik_and_open_fridge_door_server")

(defvar *dryrun* (ros::get-param "/dryrun"))
(when (null *dryrun*)
  (load "package://pr2eus/pr2-interface.l")
  (load "package://jsk_demo_common/euslisp/pr2-action.l")
  (pr2-init)
  )

(defun cb (server goal)
  (ros::ros-info "solve-ik-and-open-fridge-door callback func called")
  (cond
   (*dryrun*
    (ros::ros-info "solve-ik-and-open-fridge-door (dryrun): wait 5s")
    (unix:sleep 5)
    (ros::ros-info "solve-ik-and-open-fridge-door (dryrun): wait end")
    )
   (t
    (solve-ik-and-open-fridge-door)
    ))
  (let (msg)
    (setq msg (instance std_msgs::Bool))
    (send msg :data nil) ;; end detecting fridge door
    (ros::publish "detectfridgeflag" msg))
  (send server :set-succeeded (send server :result))
  (send *pr2* :reset-pose) ;; for another try
  )

(setq s (instance ros::simple-action-server :init
                  "solve_ik_and_open_fridge_door" roseus_smach::Sub5Action
                  :execute-cb 'cb))

(ros::advertise "detectfridgeflag" std_msgs::Bool 10)

(ros::rate 10)
(do-until-key
 (send s :worker)
  (ros::spin-once)
  (ros::sleep))

(exit)

