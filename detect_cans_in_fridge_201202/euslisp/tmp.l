#!/usr/bin/env roseus

(load "package://jsk_perception/euslisp/detection_utils.l")
(load "package://pr2eus/pr2-interface.l")
(pr2-init)

(defparameter *detection-topic* "/openni_c2/depth_registered/ObjectDetection")
(setq *use-arm-navigation* nil)
(setq *tfb* (instance ros::transform-broadcaster :init))
(defun grasp-can-single (obj &key (rotation 0) (use-arm :rarm) (grasp-depth 35)(func-time 7))
  (let (via1 via2 tar orig-av (mvt (* (/ (- func-time 4) 4.0) 1000)));; mvt is about 3000. we assume it takes 4 sec other than sending :angle-vector, and there are 4 :angle-vector in this funciton
    (send *ri* :stop-grasp use-arm)
    (send *tfb* :send-transform
          (send (send *pr2* :worldcoords) :transformation (send obj :worldcoords))
          "/base_footprint" "/objecta_pose_frame")
    (setq orig-av (send *ri* :state :potentio-vector))

    (when *use-arm-navigation*
      (dotimes (i 5)
        (send *plan-env* :get-planning-scene)
        ;; (unix::usleep (* 400 1000))
        ))

    (setq tar (send (send (send *pr2* :worldcoords) :transformation
                          (send obj :worldcoords)) :copy-worldcoords))
    (send tar :orient (deg2rad (- rotation)) :z :world)

    (setq via1 (send tar :copy-worldcoords))
    (send via1 :translate (float-vector -100 0 50))

    ;; via2
    (setq via2 (send tar :copy-worldcoords))
    (send via2 :translate (float-vector grasp-depth 0 10))

    ;; publish tf
    (send *tfb* :send-transform
          via1 "/base_footprint" "/objecta_pose_frame_via1")
    ;;(send *tfb* :send-transform
    ;;via2 "/base_footprint" "/objecta_pose_frame_via2")
    (send *tfb* :send-transform
          via2 "/base_footprint" "/objecta_pose_frame_via2")

    ;; move to via1
    (send *pr2* use-arm :inverse-kinematics
          (send via1 :transform (send *pr2* :worldcoords) :world)
          :rotation-axis t)
    (send *tfb* :send-transform
          (send (send *pr2* :worldcoords) :transformation (send *pr2* use-arm :end-coords))
          "/base_footprint" "/objecta_pose_end_coords")

    ;;(send *ri* :angle-vector (send *pr2* :angle-vector))
    ;;(send *ri* :wait-interpolation))
    ;; check IK
    (send *ri* :ros-wait 0.0 :spin-self t) ;; attention-check ...

    (cond
     ((null *use-arm-navigation*)
      (send *ri* :angle-vector (send *pr2* :angle-vector) mvt)
      (send *ri* :wait-interpolation))
     ((send *plan-env* :get-ik-for-pose (send *plan-env* :get-robot-coords use-arm) use-arm)
      (ros::ros-warn ";; can not solve collision free IK")
      (send *ri* :angle-vector (send *pr2* :angle-vector) mvt)
      (send *ri* :wait-interpolation))
     (t
      (when (send *plan-env* :planning-move-arm use-arm :planning-time 4.0)
        (ros::ros-warn ";; can not solve collision free Plan")
        (send *ri* :angle-vector (send *pr2* :angle-vector) mvt)
        (send *ri* :wait-interpolation))
      ))

    ;; move to via2 (grasping pose)
    (send *pr2* use-arm :inverse-kinematics
          (send via2 :transform (send *pr2* :worldcoords) :world)
          :rotation-axis t)
    ;; check IK
    #|(cond
    ((send *plan-env* :get-ik-for-pose (send *plan-env* :get-robot-coords :rarm) :rarm)
    (ros::ros-warn ";; can not solve collision free IK")
    ;; move to via1
    (send *ri* :angle-vector (send *pr2* :angle-vector))
    (send *ri* :wait-interpolation))
    (t
    (when (send *plan-env* :planning-move-arm :rarm :planning-time 4.0)
    (ros::ros-warn ";; can not solve collision free Plan")
    (send *ri* :angle-vector (send *pr2* :angle-vector))
    (send *ri* :wait-interpolation))
    ))|#
    (send *ri* :ros-wait 0.0 :spin-self t) ;; attention-check ...
    (send *ri* :angle-vector (send *pr2* :angle-vector) mvt)
    (send *ri* :wait-interpolation)

    ;; grasp
    (send *ri* :start-grasp use-arm)

    ;; move to via1
    (send *pr2* use-arm :inverse-kinematics
          (send via1 :transform (send *pr2* :worldcoords) :world)
          :rotation-axis t)
    ;; check IK
    (send *ri* :ros-wait 0.0 :spin-self t) ;; attention-check ...
    (cond
     ((null *use-arm-navigation*)
      (send *ri* :angle-vector (send *pr2* :angle-vector) mvt)
      (send *ri* :wait-interpolation))
     ((send *plan-env* :get-ik-for-pose (send *plan-env* :get-robot-coords use-arm) use-arm)
      (ros::ros-warn ";; can not solve collision free IK")
      ;; move to via1
      (send *ri* :angle-vector (send *pr2* :angle-vector) mvt)
      (send *ri* :wait-interpolation))
     (t
      (when (send *plan-env* :planning-move-arm use-arm :planning-time 4.0)
        (ros::ros-warn ";; can not solve collision free Plan")
        (send *ri* :angle-vector (send *pr2* :angle-vector) mvt)
        (send *ri* :wait-interpolation))
      ))
    ;;(send *ri* :angle-vector (send *pr2* :angle-vector))
    ;;(send *ri* :wait-interpolation)

    ;; move to original pose
    (send *ri* :ros-wait 0.0 :spin-self t) ;; attention-check ...
    (send *ri* :angle-vector orig-av mvt)
    (send *ri* :wait-interpolation)
    )
  )

(defun pub-cb (target-obj marker-life force msg)
  (let ((filter-obj))
    (detection-interface-objectdetection force target-obj msg)
    (cond
      ((send target-obj :stamp);;for tf error
       (setq to target-obj)
       (ros::unsubscribe *detection-topic*)
       (grasp-can-single target-obj :rotation 20 :use-arm :rarm)
       )
      (t
       nil)
      )
    )
  )



(defun only-perception (&key (timeout 10) ((:type atype) nil) (tf-force nil)
                             (diff-position 10.0)
                             (diff-rotation 1) ;; radian
                             (single nil) (speak nil)
                             (publish-objectdetection-marker t)
                             (marker-life 30)
                             (detection-topic *detection-topic*)
                             ((:target-object target-obj) (make-cube 60 60 60))
                        )
  (let ((tgt-cds (send target-obj :copy-worldcoords))
        )
    (send target-obj :reset-coords)
    (setf (get target-obj :type) atype)
    (ros::subscribe detection-topic
                  posedetection_msgs::ObjectDetection
                  #'pub-cb target-obj marker-life tf-force))
    (ros::rate 10)
    (ros::spin)
)

(only-perception)

