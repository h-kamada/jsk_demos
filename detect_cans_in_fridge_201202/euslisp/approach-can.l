(ros::load-ros-manifest "roseus_smach")
(ros::roseus "approach_can_server")

(defvar *dryrun* (ros::get-param "/dryrun"))
(when (null *dryrun*)
  (load "package://pr2eus/pr2-interface.l")
  (load "package://jsk_demo_common/euslisp/pr2-action.l")
  (pr2-init)
  )

(defun cb (server goal)
  (ros::ros-info "approach-can callback func called")
  (cond
   (*dryrun*
    (ros::ros-info "approach-can (dryrun): wait 5s")
    (unix:sleep 5)
    (ros::ros-info "approach-can (dryrun): wait end")
    )
   (t
    (approach-can :use-arm nil)
    ))
  (send server :set-succeeded (send server :result))
  (send *pr2* :reset-pose) ;; for another try
  )

(setq s (instance ros::simple-action-server :init
                  "approach_can" roseus_smach::Sub5Action
                  :execute-cb 'cb))

(ros::rate 10)
(do-until-key
 (send s :worker)
  (ros::spin-once)
  (ros::sleep))

(exit)

