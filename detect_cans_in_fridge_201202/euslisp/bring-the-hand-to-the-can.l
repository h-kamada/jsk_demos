(ros::load-ros-manifest "roseus_smach")
(ros::roseus "bring_the_hand_to_the_can_server")

(defvar *dryrun* (ros::get-param "/dryrun"))
(when (null *dryrun*)
  (load "package://pr2eus/pr2-interface.l")
  (load "package://jsk_demo_common/euslisp/pr2-action.l")
  (pr2-init)
  )

(defun cb (server goal)
  (ros::ros-info "bring-the-hand-to-the-can callback func called")
  (cond
   (*dryrun*
    (ros::ros-info "bring-the-hand-to-the-can (dryrun): wait 5s")
    (unix:sleep 5)
    (ros::ros-info "bring-the-hand-to-the-can (dryrun): wait end")
    )
   (t
    (bring-the-hand-to-the-can)
    ))
  (send server :set-succeeded (send server :result))
  (send *pr2* :reset-pose) ;; for another try
  )

(setq s (instance ros::simple-action-server :init
                  "bring_the_hand_to_the_can" roseus_smach::Sub5Action
                  :execute-cb 'cb))

(ros::rate 10)
(do-until-key
 (send s :worker)
  (ros::spin-once)
  (ros::sleep))

(exit)

