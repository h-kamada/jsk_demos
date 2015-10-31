
(ros::load-ros-manifest "roseus_smach")
(ros::roseus "goto_front_of_fridge_server")

(load "package://pr2eus/pr2-interface.l")
;(load "package://jsk_demo_common/euslisp/pr2-move.l")
;(pr2-init)

(defun cb (server goal)
  (ros::ros-info "goto-front-of-fridge callback func called")

  ;;  (goto-front-of-fridge)
  (ros::ros-info "wait 0.5s")
  (unix:usleep 500)
  (send server :set-succeeded (send server :result :value 1))
  )

(setq s (instance ros::simple-action-server :init
                  "goto_front_of_fridge" roseus_smach::Sub5Action
                  :execute-cb 'cb))

(ros::rate 10)
(do-until-key
 (send s :worker)
  (ros::spin-once)
  (ros::sleep))

(exit)


