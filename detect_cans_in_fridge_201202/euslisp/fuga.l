
(ros::load-ros-manifest "roseus_smach")
(ros::roseus "fugaserver")

(load "package://pr2eus/pr2-interface.l")
(pr2-init)

;; callbackで、succeedしたことを記述しないと、client側が待ちつづけてしまう
(defun fuga-callback (server goal)
  (ros::ros-info "fugafuga callbacked")

  (send *ri* :angle-vector #f(49.8741 31.5769 -15.4598 36.0961 -53.6821 -4.53289 -5.88339 172.813 -11.1008 -20.1584 16.1525 -52.8526 -16.0577 -29.8038 180.014 7.46612 -21.2682)
      3000 :rarm-controller)

  (setq msg (send server :result :value 1))
  (send server :set-succeeded msg)
  )

(setq s (instance ros::simple-action-server :init
                  "fugafuga" roseus_smach::Sub5Action
                  :execute-cb 'fuga-callback))

(ros::rate 10)
(do-until-key
 (send s :worker)
  (ros::spin-once)
  (ros::sleep))

(exit)



