

(load "package://roseus_smach/src/state-machine-actionlib.l")

(defclass fm
    :super propertied-object
  :slots ())
(defmethod fm
    (:init ())
  (:m () (ros::rosinfo "hoge")))

(defun make (hoge)
  (funcall
   `(lambda ()
      (let ((sm (instance fm :init))
            (name (format nil ":~A" ,hoge))
            )
        (ros::ros-info "name : ~A" name)
        (setq *sm* sm)
        (setq *n* name)
        (send sm name)
        )
      )))



