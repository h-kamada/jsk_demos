(setq *obj* (make-sphere 100)) ;; dummy object
(setq *target-name* nil)
(defparameter *detection-topic* "/kinect_head/rgb/ObjectDetection")
(require :detection-interface "package://jsk_perception/euslisp/detection_interface.l")
(require :pr2-pose "package://jsk_demo_common/euslisp/pr2-pose.l")


;; check if jsk_smart_gui exists
(if (ros::rospack-find "jsk_smart_gui")
    (progn
      (ros::load-ros-manifest "jsk_smart_gui")
      (require :jsk-smart-gui-utils "package://jsk_smart_gui/src/utils.l"))
    (warning-message 1 ";; Could not found jsk_smarg_gui....~%"))

(defvar *use-voicetext* nil)
(defvar *use-english* nil)

(if (and (boundp '*use-voicetext*) *use-voicetext*)
    (cond
     ((and (boundp '*use-english*) *use-english*)
      (setq *frige-speak-str* "the refrigerator"))
     (t (setq *frige-speak-str* "冷蔵庫")))
  (setq *frige-speak-str* "れいぞうこ"))

;;
;; action-utility
;;

(defun pick (pos &key (arm :larm)
			 (debug-view nil))
  (let ((tmp-pos (send pos :copy-worldcoords)))
    (format t  "Start Pick: ~a~%" pos)
    (send *ri* :stop-grasp arm)  ;; Stop grasp
    (send *pr2* :inverse-kinematics 
		  (progn
			(send tmp-pos :rotate  (deg2rad 90) :y) ;; alignment
			(send tmp-pos :translate #f(-25 0 0)))  ;; avoid collision
		  :rotation-axis :x
		  :move-target (send *pr2* arm :end-coords)
		  :link-list (send *pr2* :link-list
						   (send *pr2* arm :end-coords :parent)
						   (send *pr2* :link "torso_lift_link"))
		  :debug-view nil)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
    (send *ri* :wait-interpolation)
    (unix::usleep 100000)

    ;; Low down to pick 
    (send *pr2* :torso :waist-z
		  :joint-angle (- (send *pr2* :torso :waist-z :joint-angle) 30))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
    (send *ri* :wait-interpolation)
    (unix::usleep 100000)
    (send *ri* :start-grasp arm)
    ;;(send *ri* :move-gripper arm 0.0 :effort 100 :wait t)
    (print "End Pick!"))
  )

(defun pick2 (pos &key (arm :rarm)
			  (rotation-axis :z)
			  (debug-view nil))
  (let ((tmp-pos (send pos :copy-worldcoords)))
    (format t  "Start Pick: ~a~%" pos)
    (send *ri* :stop-grasp arm)  ;; Stop grasp
    (send *pr2* arm :inverse-kinematics 
		  tmp-pos
		  :rotation-axis rotation-axis
		  :debug-view nil)
	(send *pr2* arm :move-end-pos (float-vector -100 0 0))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
    (send *ri* :wait-interpolation)
    (unix::usleep 100000)

	(send *pr2* arm :move-end-pos (float-vector 150 0 0))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
    (send *ri* :wait-interpolation)
    (unix::usleep 100000)

;    (send *pr2* :torso :waist-z
;		  :joint-angle (+ (send *pr2* :torso :waist-z :joint-angle) 230))
;    (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
;    (send *ri* :wait-interpolation)
;    (unix::usleep 100000)
    (send *ri* :start-grasp arm)
    ;;(send *ri* :move-gripper arm 0.0 :effort 100 :wait t)
    (print "End Pick!"))
  )

(defun place2 (pos &key (arm :rarm)
			  (rotation-axis :z)
			  (debug-view nil))
  (let ((tmp-pos (send pos :copy-worldcoords)))
	(send *pr2* arm :inverse-kinematics
		  tmp-pos
		  :rotation-axis rotation-axis
		  :debug-view debug-view)
	(send *pr2* :torso :waist-z :joint-angle
		  (+ (send *pr2* :torso :waist-z :joint-angle) 100))
	(send *ri* :angle-vector (send *pr2* :angle-vector))
	(send *ri* :wait-interpolation)
	
	(send *pr2* :torso :waist-z :joint-angle
		  (- (send *pr2* :torso :waist-z :joint-angle) 100))
	(send *ri* :angle-vector (send *pr2* :angle-vector))
	(send *ri* :wait-interpolation)
	(send *ri* :stop-grasp arm)
	(print "End Place!"))
  )

(defun place (pos &key (arm :larm)
			  (height -50)
			  (debug-view nil))
  (let ((tmp-pos (send pos :copy-worldcoords)))
    (format t  "Start Place: ~a~%" pos)
    (send *pr2* :inverse-kinematics 
	  (progn 
	    (send tmp-pos :rotate (deg2rad 90) :y)
	    (send tmp-pos :translate (float-vector height 0.0 0.0)))
	  :rotation-axis :x  ;; 
	  :move-target (send *pr2* arm :end-coords)
	  :link-list (send *pr2* :link-list
			   (send *pr2* arm :end-coords :parent)
			   (send *pr2* :link "torso_lift_link"))
	  :debug-view debug)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
    (send *ri* :wait-interpolation)
    (unix::usleep 100000)
    
    (send *pr2* :torso :waist-z
     	  :joint-angle (- (send *pr2* :torso :waist-z :joint-angle) 30))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
    (send *ri* :wait-interpolation)
    (unix::usleep 100000)

    (send *ri* :stop-grasp arm)
    
    (send *pr2* :torso :waist-z
	  :joint-angle (- (send *pr2* :torso :waist-z :joint-angle) -50))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
    (send *ri* :wait-interpolation)
    (unix::usleep 100000)
    (print "End Place!"))
  )

(defmacro start-detect-tabletop-object ()
  `(progn
	 (if (not (boundp '*tfb*)) (setq *tfb* (instance ros::transform-broadcaster :init)))
	 (defun tabletop-objectdetection-cb (msg)
	   (let ((frame-id (send msg :header :frame_id)) atype ret)
		 (print msg)
		 (unless (eq (char frame-id 0) #\/) (setq frame-id (concatenate string "/" frame-id)))
		 (dolist (obj-pose (send msg :objects))
		   (setq atype (send obj-pose :type))
		   (unless (eq (char atype 0) #\/) (setq atype (concatenate string "/" atype)))
		   (setq ret (ros::tf-pose->coords (send obj-pose :pose)))
		   (send *tfb* :send-transform ret frame-id atype)
		   (ros::ros-info "~A ~A ~A" ret frame-id atype))
		 ))
	 (ros::subscribe "ObjectDetection" posedetection_msgs::ObjectDetection #'tabletop-objectdetection-cb))
  )


(defun stop-detect-tabletop-object ()
  (ros::unsubscribe "ObjectDetection"))

(defun detect-tabletop-object (&optional (root-frame-id "/base_footprint"))
  (ros::spin-once)
  (if (not (boundp '*tfl*)) (setq *tfl* (instance ros::transform-listener :init)))
  (let (c obj-frame-id ret)
    (dolist (obj-frame-id (list "/tabletop0"
								"/tabletop1"
								"/tabletop2"
								"/tabletop3"
								"/tabletop4"))
      ;; advertise
      (when (send *tfl* :get-latest-common-time root-frame-id obj-frame-id)
		(setq c (send *tfl* :lookup-transform root-frame-id obj-frame-id (ros::time 0)))
		(when c
		  (ros::ros-info "~A ~A" obj-frame-id c)
		  (push c ret))))
	(ros::sleep)
    ret))

(defparameter *detection-loop-threshold* 0)
(warn ";; define detect-with-base-laser")
(defun detect-with-base-laser (obj obj-type &key (arg nil) (debug nil) (speak nil) (publish-marker nil))
  (ros::ros-info "detect-with-base-laser~%")
  (unless (not (send *ri* :simulation-modep))
	(if obj (ros::ros-warn "~A detection failed" obj))
    (return-from detect-with-base-laser nil)
    )
  (let ((loop-flag t) (loop-counter 0) (obj-coords nil) (ret))
    (ros::roseus "objectdetection_publisher")
    (defparameter *detection-topic* "/narrow_stereo/left/ObjectDetection")
    (ros::rate 10)
    (ros::ros-info "detecting_now")
    (while loop-flag
      (setq ret (check-detection :type obj-type ;; work for any object
                                 :speak-name (if speak speak "")
                                 :target-object obj
                                 :publish-objectdetection-marker publish-marker
                                 :speak speak))
      (ros::spin-once)
      (if ret
          (progn
            (ros::ros-info "object pos info ~A before trans" obj)
            (send obj :translate (send *pr2* :worldpos) :world)
            (ros::ros-info "object pos info ~A after trans" obj)
            (send *pr2* :head :look-at
				  (if spot
					  (send (send obj spot) :worldpos)
					  (send obj :worldpos)))
            (send *ri* :angle-vector (send *pr2* :angle-vector))
            (push (send obj :worldcoords) obj-coords)
            ))

      ;;wait a few loop
      (when (> loop-counter *detection-loop-threshold*)
        (setq loop-flag nil)	;;temp method
        )
      (inc loop-counter)
      (ros::ros-info "~d loop counter ----" loop-counter)
      (ros::sleep)
      )
    )
  )

(warn ";; define hold-chair")
(defun hold-chair (chair)
  (hold-chair-prepair chair)
  (send *pr2* :inverse-kinematics (send (send *pr2* :larm :end-coords :copy-worldcoords) :translate #f(-300 0 0) *pr2*) :move-arm :larm :use-torso nil :rotation-axis :y)
  (send *pr2* :inverse-kinematics (send (send *pr2* :rarm :end-coords :copy-worldcoords) :translate #f(-300 0 0) *pr2*) :move-arm :rarm :use-torso nil :rotation-axis :y)
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation)
  (send *pr2* :rarm :angle-vector #f(-14.3227 16.0059 -85.3758 -102.121 -35.757 -32.3901 -173.201))
  (send *pr2* :larm :angle-vector #f(8.64394 19.7097 109.218 -85.0018 43.0966 -7.98218 163.739))
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation))

(defun hold-chair-prepair (chair)
  (send *pr2* :larm :angle-vector #f(25 0 0 -130 0 0 0))
  (send *pr2* :rarm :angle-vector #f(-25 0 0 -130 0 0 0))
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation)
  (send *pr2* :reset-pose)
  (send *pr2* :head :look-at (vector-mean (send-all (send chair :handle) :worldpos)) :world)
  (send-message *pr2* euscollada-robot
				:inverse-kinematics (send (send (send (send chair :handle-larm) :copy-worldcoords) :rotate -pi/2 :x) :translate #f(30 100 200))
				:rthre (deg2rad 5)
				:thre 10
				:stop 300
				:weight #f(0 1 1 1 1 1 1 0)
				:link-list (send *pr2* :link-list (send (send *pr2* :larm :end-coords) :parent))
				:move-target (send *pr2* :larm :end-coords)
				:rotation-axis :y)
  (send-message *pr2* euscollada-robot
				:inverse-kinematics (send (send (send (send chair :handle-rarm) :copy-worldcoords) :rotate pi/2 :x) :translate #f(30 -100 200))
				:rthre (deg2rad 5)
				:thre 10
				:stop 300
				:weight #f(0 1 1 1 1 1 1 0)
				:link-list (send *pr2* :link-list (send (send *pr2* :rarm :end-coords) :parent))
				:move-target (send *pr2* :rarm :end-coords)
				:rotation-axis :y)
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation)
  (send-message *pr2* euscollada-robot
				:inverse-kinematics (send (send (send (send chair :handle-larm) :copy-worldcoords) :rotate -pi/2 :x) :translate #f(30 100 -120))
				:rthre (deg2rad 5)
				:thre 10
				:stop 300
				:weight #f(0 1 1 1 1 1 1 0)
				:link-list (send *pr2* :link-list (send (send *pr2* :larm :end-coords) :parent))
				:move-target (send *pr2* :larm :end-coords)
				:rotation-axis :y)
  (send-message *pr2* euscollada-robot
				:inverse-kinematics (send (send (send (send chair :handle-rarm) :copy-worldcoords) :rotate pi/2 :x) :translate #f(30 -100 -120))
				:rthre (deg2rad 5)
				:thre 10
				:stop 300
				:weight #f(0 1 1 1 1 1 1 0)
				:link-list (send *pr2* :link-list (send (send *pr2* :rarm :end-coords) :parent))
				:move-target (send *pr2* :rarm :end-coords)
				:rotation-axis :y)
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation))

(defun open-fridge-traj (hand handle &optional (rad/length pi/2) &rest args
                         &key (radius 500) (path-div 20) (door-type :circle) &allow-other-keys)
  (let ((cds-traj
         (case door-type
           (:circle
            (open-fridge-circle-traj handle rad/length :radius radius :path-div path-div))
           (:slide
            (open-fridge-slide-traj handle rad/length :path-div path-div))
           )))
    (apply #'move-fridge-traj hand cds-traj args)
    ))
(defun open-fridge-slide-traj
  (handle &optional (path-length 200)
          &key  (path-div 20) (pre-start-x -100) (start-x 25)
          &allow-other-keys)
  (let ((tcds (send (send handle :copy-worldcoords) :translate (float-vector (- path-length) 0 0)))
        ret)
    (setq ret (generate-slide-path (send handle :worldpos)
                                   (send handle :worldpos) (send tcds :worldpos)
                                   path-length :div path-div))
    (mapc #'(lambda (x) (send x :rotate pi/2 :x)) ret)
    (setq *ret*
          (append (list (send (send (car ret) :copy-worldcoords)
                              :translate (float-vector pre-start-x 0 0) :local)
                        (send (send (car ret) :copy-worldcoords)
                              :translate (float-vector start-x 0 0) :local))
                  ret))
    ;; stop grasp
    ;; move (elt ret 0)
    ;; move (elt ret 1)
    ;; start grap
    ;; grasp check
    ;; move-trajectory (subseq ret 2)
    ;; finalize
    *ret*))

(defun open-fridge-circle-traj
  (handle &optional (rad pi/2)
          &key (radius 500) (path-div 20) (pre-start-x -100) (start-x 25))
  ;; 500mm is a distance between handle and axis
  (let ((acds (send (send handle :copy-worldcoords) :translate (float-vector 0 (- radius) 200)))
        (bcds (send (send handle :copy-worldcoords) :translate (float-vector 0 (- radius) -200)))
        (di -pi/2) ret)
    (setq ret (generate-circle-path (send handle :worldpos)
                                    (send acds :worldpos) (send bcds :worldpos)
                                    rad :div path-div));;a,b can be reverse
    (mapc #'(lambda (x) (send x :rotate -pi/2 :x)) ret)
    (setq *ret*
          (append (list (send (send (car ret) :copy-worldcoords)
                              :translate (float-vector pre-start-x 0 0) :local)
                        (send (send (car ret) :copy-worldcoords)
                              :translate (float-vector start-x 0 0) :local))
                  ret))
    ;; stop grasp
    ;; move (elt ret 0)
    ;; move (elt ret 1)
    ;; start grap
    ;; grasp check
    ;; move-trajectory (subseq ret 2)
    ;; finalize
    *ret*))

(defun move-fridge-traj (hand cds-traj
                              ;; &key ((:rotation-axis ra) :z) (use-torso 0.0025)
                              &key ((:rotation-axis ra) :z) (use-torso nil)
                              (fix-waist-z 130) (return-sequence) (wait-time 1)
                              (grasp-check t)  (move-robot t) (start-sec 1000) (func-time 20) &allow-other-keys)
  (let* ((st0 (car cds-traj))
	 (st1 (cadr cds-traj))
	 (lst (cddr cds-traj))
	 (time-tick (/ (* (- func-time 16) 1000) (length lst)));;changing opening-door speed
	 avs tms)
    ;; (when move-robot  ;; open-gripper
    ;;   (send *ri* :move-gripper hand 0.09 :wait nil))
    (when fix-waist-z
      (unless (numberp fix-waist-z) (setq fix-waist-z 130))
      (send *pr2* :torso :waist-z :joint-angle fix-waist-z))
    ;; (cond
    ;;  ((move-arm hand st0
    ;;             :use-torso (if use-torso (/ use-torso 5))
    ;;             :rotation-axis t :sec start-sec :move-robot move-robot)
    ;;   t)
    ;;  (t
    ;;   (ros::ros-warn "Could not solve ik, return from execute-open")
    ;;   (return-from move-fridge-traj nil)))

    ;; (move-arm hand st1
    ;;           :use-torso (if use-torso (/ use-torso 5))
    ;;           :rotation-axis t :sec start-sec :move-robot move-robot)

    ;; below takes 1s But ik time short so grasp again
    ;; (warning-message 4 "[~A] a_a~%" (ros::time-now))
    ;; (let ((grasp-ret (if move-robot (send *ri* :start-grasp hand :gain 0.1 :objects nil) 10)))
    ;;   (when (and (not (send *ri* :simulation-modep)) ;;return only in real mode
    ;;              (and grasp-check
    ;;                   (< grasp-ret 8))) ;; grasp
    ;;     (ros::ros-warn "Grasp handle failed, return from execute-open")
    ;;     (return-from move-fridge-traj nil)))

    (let (start-time)
      (warning-message 4 "[~A] a~%" (ros::time-now))
      (setq start-time (ros::time-now))
      (dolist (rr lst)
        (let ((ik-solved
               (send *pr2* :inverse-kinematics
                     (send rr :worldcoords)
                     :use-torso use-torso :stop 10 ;; end time change
                     :move-arm hand :rotation-axis ra :debug-view nil :warnp nil)))
          (unless (or ik-solved (eq ra nil))
                  (warn ";; retry ~A~%" rr);
                  (setq ik-solved
                        (send *pr2* :inverse-kinematics
                              (send rr :copy-worldcoords)
                              :use-torso use-torso
                              :move-arm hand :rotation-axis nil :debug-view nil :warnp nil)))
          (unless ik-solved (warn ";; failed ~A~%" rr))
          (warning-message 4 "solve ik[~A]~%" (ros::time-now))
          (when ik-solved
            (push (send *pr2* :angle-vector) avs)
            (push time-tick tms))
          ))
      (warning-message 4 "time:~A b~%" (ros::time-now))
      (warning-message 4 "ik solve time:~A~%" (ros::time- (ros::time-now) start-time))
      )
    (when return-sequence
      (return-from move-fridge-traj (list (reverse avs) tms)))

    ;; (unix:sleep 3) ;; temporary in parallel mode only. remove in sequential
    ;; ik doesn't take so much time now. delete
    (when move-robot
      ;; (send *ri* :ros-wait 0.0 :spin-self t) ;; attention-check ...
      ;; add rarm-controller 直すべき
      (send *ri* :angle-vector-sequence (reverse avs) tms :rarm-controller))
    ;; finalize
    (warning-message 4 "time:~A c~%" (ros::time-now))
    (cond
     ((integerp wait-time)
      (send *ri* :ros-wait wait-time :spin-self t)) ;; attention-check ...
     ((numberp wait-time)
      (send *ri* :ros-wait wait-time :spin-self t) ;; attention-check ...
      ;; (unix::usleep
      ;; (round (* wait-time 1000 1000)))
      )
     (wait-time
      (send *ri* :wait-interpolation)))
    ;; (send *ri* :stop-grasp hand)
    ;; (send *ri* :wait-interpolation)
    ;; ;;
    ;; (send *ri* :ros-wait 0.0 :spin t :spin-self t) ;; attention-check ...
    (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
    (warning-message 4 "d~%")
    (let ((end-pt (send *pr2* hand :end-coords :worldpos))
          idx)
      (setq idx (position (find-extream lst #'(lambda(x) (distance (send x :worldpos) end-pt)) #'<) lst))
      (ros::ros-info "idx = [~a/~a]" idx (length lst))
      (if (send *ri* :simulation-modep) (return-from move-fridge-traj t))
      (when (< idx (/ (length lst) 2))
        (return-from move-fridge-traj nil)))
    t))

;;
;; actions
;;
(warn ";; define grasp-can-single")
(defun grasp-can-single (obj &key (rotation 0) (use-arm :larm) (grasp-depth 35)(func-time 7))
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

    (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
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
    (send *ri* :start-grasp use-arm :objects nil)

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

(defun pr2-look-around (&key (time-tick 600) (show-wait 1200)
                             (yaw-angle-list (list 16 -16))
                             (pitch-angle-list (list 14 48))
                             (waist-z 140) (move-to-initial t))
  (let ((last-angle (send *pr2* :angle-vector))
        (y-angle yaw-angle-list)
        (p-angle pitch-angle-list))
    (send *pr2* :head :angle-vector (float-vector (car y-angle) (car p-angle)))
    (when move-to-initial
      (send *pr2* :rarm :angle-vector #f(-80.0 74.0 -20.0 -120.0 -20.0 -30.0 180.0))
      (send *pr2* :larm :angle-vector #f(80.0 74.0 20.0 -120.0 20.0 -30.0 180.0)))
    (send *pr2* :torso :angle-vector (float-vector waist-z))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1400)
    (send *ri* :wait-interpolation)

    (dolist (ya y-angle)
      (dolist (pa p-angle)
        (send *pr2* :head :angle-vector (float-vector ya pa))
        (send *ri* :angle-vector (send *pr2* :angle-vector) time-tick)
        (send *ri* :wait-interpolation)
        (unix::usleep (round (* show-wait 1000)))
        )
      (setq p-angle (nreverse p-angle)))

    (send *pr2* :angle-vector last-angle)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1400)
    (send *ri* :wait-interpolation)
    ))

(defun open-fridge-door-initial-pose (&key (wait t) (door-type :circle) (head-pitch 14) (torso-lift 130) (ratio 1))

  (case door-type
    (:circle
     (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
     (send *pr2* :head :angle-vector (float-vector 0 head-pitch))
     (send *pr2* :torso :angle-vector (float-vector torso-lift))
     (send *ri* :angle-vector (send *pr2* :angle-vector) (* ratio 3000))
     (send *ri* :stop-grasp)
     (if wait
         (send *ri* :wait-interpolation))
     )
    ((:slide1 :slide2)
     (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
     (send *pr2* :head :angle-vector (float-vector 0 head-pitch))
     (send *pr2* :torso :angle-vector (float-vector torso-lift))
     ;;(send *pr2* :larm :angle-vector (float-vector 25.0 74.0 35.0 -120.0 20.0 -30.0 0.0))
     (send *ri* :angle-vector (send *pr2* :angle-vector) (* ratio 3000))
     (send *ri* :wait-interpolation)
     ))
  )

(defun preset (&optional (move nil))
  (send *pr2* :reset-pose)
  (send *pr2* :torso :waist-z :joint-angle 130)
  (when move
    (send *ri* :angle-vector (send *pr2* :angle-vector))
    ))

(defun treset (&optional (move nil))
  (send *pr2* :reset-pose)
  (when move
    (send *ri* :angle-vector (send *pr2* :angle-vector))
    ))


(defparameter *fridge-handle-cds* nil)
(defun approach-fridge-door (&key (door-type :circle) (ratio 1) (look-around nil) (head-pitch 14) (torso-lift 130) (move t) (open-fridge-func #'open-fridge-traj) (wait t))
  (let (ret
        (idealcds
         (case door-type
           (:circle
            (make-coords :pos (float-vector 775 100 0)
                         ;;(float-vector 790 100 0) ;;(float-vector 760 100 0)
                         :rpy (float-vector 0.05 0 0)))
           (:slide1
            (make-coords :pos (float-vector 780 100 0)
                         :rpy (float-vector 0.0 0 0)))
           (:slide2
            (make-coords :pos (float-vector 750 100 0)
                         :rpy (float-vector 0.0 0 0)))))
        (diffcds (make-coords :pos (float-vector 10000 10000 10000)))
        cds
        )
    ;; finding handle position
    (cond
     ((not (send *ri* :simulation-modep)) ;; real mode
      (setq cds (check-detection-srv :type "fridge" :speak-name *frige-speak-str*
                                 :detection-topic "/kinect_head/rgb/ObjectDetection"
                                 )))
     (t ;; simulation mode
      (setq cds (send (make-cube 60 60 60) :translate (float-vector 777 98 1112)))
      ))
    ;; (setq *fridge-handle-cds* cds)
    ;; ;; need detection fail check
    ;; (send *ri* :ros-wait 0.0 :spin-self t) ;; attention-check ...
    (when cds
      (send *obj* :reset-coords)
      (send *obj* :transform cds)
      (dump-structure *standard-output*
                      (list (send *pr2* :copy-worldcoords)
                            (send cds :copy-worldcoords)))
      (setq cds
            (make-coords :pos
                         (float-vector (elt (send cds :worldpos) 0)
                                       (elt (send cds :worldpos) 1) 0)
                         :rpy (list (elt (car (rpy-angle (send cds :worldrot))) 0)
                                    0 0)))

      (setq diffcds (send (send idealcds :inverse-transformation)
                          :transform (send cds :worldcoords) :world))

      (warn "~%~A -> ~A / ~A~%" diffcds cds idealcds)
      (ros::ros-info "DIFF: ~A" diffcds)
      (cond
       ((or (send *ri* :simulation-modep) ;;if in simulation mode, execute
            (and (< (norm (float-vector (elt (send diffcds :worldpos) 0)
                                        (elt (send diffcds :worldpos) 1)))
                    *fridge-distance-threshold*)
                 (< (abs (elt (car (rpy-angle (send diffcds :worldrot))) 0))
                    *fridge-rotation-threshold*)))
        (cond
         ((not (send *ri* :simulation-modep)) ;; real mode
          (setq cds (check-detection-srv :type "fridge" :speak-name *frige-speak-str*
                                     :detection-topic "/kinect_head/rgb/ObjectDetection"))
          )
         (t ;; simulation mode
          (setq cds (send (make-cube 60 60 60) :translate (float-vector 777 98 111212)))))
        ;; (send *ri* :ros-wait 0.0 :spin-self t :spin t) ;; attention-check ...
        ;; (y-or-n-p "Can I start open fridge? ")
        (when cds
          (if look-around (pr2-look-around))
          ;; for open fridge
          (case door-type
            (:circle
             ;; (send *pr2* :head :angle-vector (float-vector 0 24))
             ;; (send *pr2* :torso :angle-vector (float-vector 130))
             ;; (send *ri* :angle-vector (send *pr2* :angle-vector) (* ratio 3000))
             ;; (send *ri* :wait-interpolation)
             )
            ((:slide1 :slide2)
             (send *pr2* :head :angle-vector (float-vector 0 50))
             (send *pr2* :torso :angle-vector (float-vector 130))
             (send *ri* :angle-vector (send *pr2* :angle-vector) (* ratio 1000))
             (send *ri* :wait-interpolation)
             ))

          (setq cds
                (make-coords :pos (send cds :worldpos)
                             :rpy (list (caar (send cds :rpy-angle)) 0 0)))
          (case door-type
            (:slide1
             (send cds :translate (float-vector 0 -230 -355)))
            (:slide2
             (send cds :translate (float-vector 0 -230 -675))))
          (setq *last-fridge-handle* cds)
          ;; pr2 local coords transform to worldcoords
          (send cds :transform (send *pr2* :worldcoords) :world)
          ;; keep head, torso and gripper, but start from reset-pose
          (let ((prev-av (send *pr2* :angle-vector))
                (hand-av))
            (send *pr2* :reset-pose)
            (setq hand-av (send *pr2* :rarm :angle-vector))
            (send *pr2* :angle-vector prev-av)
            (send *pr2* :torso :waist-z :joint-angle 130) ;; torso 動かないように
            (send *pr2* :rarm :angle-vector hand-av))
          (case door-type
            (:circle
             (cond
              (;; move-arm はcontroller全てに送ってしまうので、こう書き換えた
               ;; これによりheadが動くのも防ぐ
               (send *pr2* :inverse-kinematics
                     (send (send cds :copy-worldcoords) :translate (float-vector -100 0 0) :local)
                     :move-arm :rarm :rotation-axis t :use-torso nil :revert-if-fail nil
                     :debug-view nil)
               (send *ri* :angle-vector (send *pr2* :angle-vector) 1500 :rarm-controller)
               (send *ri* :wait-interpolation)
               ;; (move-arm :rarm (send (send cds :copy-worldcoords) :translate (float-vector -100 0 0) :local)
               ;;         :use-torso nil
               ;;         :rotation-axis t :sec 1000 :move-robot t)
               t)
              (t
               (ros::ros-warn "Could not solve ik, return from approach-fridge-door")
               (return-from approach-fridge-door nil)))
             (setq ret t)
             ;; (setq ret (funcall open-fridge-func
             ;;                    :rarm cds 1.7 ;;(/ pi 2)
             ;;                    :rotation-axis t :radius 490
             ;;                    :wait-time 5.8 ;; t
             ;;                    )))
             )
            ((:slide1 :slide2)
             (cond
              ((move-arm :rarm (send (send cds :copy-worldcoords) :translate (float-vector -100 0 0) :local)
                       :use-torso nil
                       :rotation-axis t :sec 1000 :move-robot t)
               t)
              (t
               (ros::ros-warn "Could not solve ik, return from approach-fridge-door")
               (return-from approach-fridge-door nil)))
             (setq ret t)
             ;; (setq ret (funcall open-fridge-func
             ;;                    :rarm cds 320 ;; 330 ;; max
             ;;                    :rotation-axis t :door-type :slide
             ;;                    :wait-time t
             ;;                    )))
             )
            )
          ;; if -> when で何か変わった？？
          (when ret
            (warning-message 2 "return")
            (return-from approach-fridge-door cds)))
        (if (and (boundp '*use-voicetext*) *use-voicetext*)
            (cond
             ((and (boundp '*use-english*) *use-english*)
              (speak-jp "I could not open the refrigerator."))
             (t (speak-jp "冷蔵庫を開けられませんでした。")))
          (speak-jp "れいぞうこを あけられません でした"))
        (send *pr2* :head :angle-vector (float-vector 0 head-pitch))
        (send *pr2* :torso :angle-vector (float-vector torso-lift))
        (send *ri* :angle-vector (send *pr2* :angle-vector) (* ratio 3000))
        (send *ri* :go-pos-unsafe -0.1 0 0)
        (send *ri* :wait-interpolation)
        )
       (t ;;  there is much difference
        (when move
            (cond
             (wait
              (send *ri* :go-pos-unsafe
                    (/ (elt (send diffcds :worldpos) 0) 1100.0)
                    (/ (elt (send diffcds :worldpos) 1) 1100.0)
                    (* 0.9 (rad2deg (elt (car (rpy-angle (send diffcds :worldrot))) 0))))
              ;; wait ???
              ;; (send *ri* :ros-wait 0.5 :spin-self t :spin t) ;; attention-check ...
              ;; (send *ri* :wait-interpolation)
              )
             (t
              (send *ri* :go-pos-unsafe-no-wait
                    (/ (elt (send diffcds :worldpos) 0) 1100.0)
                    (/ (elt (send diffcds :worldpos) 1) 1100.0)
                    (* 0.9 (rad2deg (elt (car (rpy-angle (send diffcds :worldrot))) 0))))
              )))
        ))
      ) ;; /when cds
    ret
    ))

;; overwrite approach-fridge-door
(defun approach-fridge-door (&key (move t) (use-arm :rarm) (timeout 20)) ;; approach-to-can
  ;; make sure torso joint-angle is 130! If not, PR2 will grasp under point
  (let (tmp-cds
        (pos-offset (float-vector -780 -100 0)) ;; 750だとswipeでかする y-100 でswipeあてる
        (distance-thre 800)
        (y-thre 100) ;; 追っかけ手の動き幅(y)
        (default-cds (float-vector 600 0 1160))
        (end-time (ros::time+ (ros::time-now) (ros::time timeout)))
        (ratio 0.9)
        )
    (while t
      (when (ros::time> (ros::time-now) end-time) ;; timeout
        (return-from approach-fridge-door nil))
      ;; detection
      (setq cds (check-detection-srv
                 :type "fridge"
                 :detection-topic "/kinect_head/rgb/ObjectDetection"))
      ;; (setq cds (send (make-cube 60 60 60) :translate (float-vector 777 98 1112))) ;; temporary

      ;; fix rot
      (setq tmp-cds (send cds :copy-worldcoords))
      (setq tmp-cds
            (make-coords :pos (send tmp-cds :worldpos)
                         :rpy (list (caar (send tmp-cds :rpy-angle)) 0 0)))
      (send tmp-cds :transform (send *pr2* :worldcoords) :world)
      (setq *cds* tmp-cds)
      ;; approach- can version so not use
      ;; (setq tmp-cds (send (send (send *pr2* :worldcoords) :transformation
      ;;                           (send cds :worldcoords)) :copy-worldcoords))
      ;; (send tmp-cds :orient 0 :z :world)

      ;; keep head, torso and gripper, but start from reset-pose
      (let ((prev-av (send *pr2* :angle-vector))
            (hand-av))
        (send *pr2* :reset-pose)
        (setq hand-av (send *pr2* :rarm :angle-vector))
        (send *pr2* :angle-vector prev-av)
        (send *pr2* :torso :waist-z :joint-angle 130) ;; torso 動かないように
        (send *pr2* :rarm :angle-vector hand-av))
      ;; approach- can version not use
      ;; for inverse-kinematics initial pose
      ;; (send *pr2* :reset-pose)
      ;; (send *pr2* :torso :waist-z :joint-angle 130)

      (warning-message 2 "go-waitp: ~A~%" (send *ri* :go-waitp))
#|
this version is temporary
because  :go-waitp isn't working well currently.
      when use-arm
        send angle to arm
      if moving
        nil
      if not moving
        when in region
          when use-arm
            wait interpolation
            send angle to arm (to back front)
          return from func
        when move
          go-pos

let's go. temporary function below
|#
      (ros::ros-info "tmp-cds:~A" tmp-cds)
      (when use-arm
        (let ((tv (send (send tmp-cds :copy-worldcoords) :pos))
              (dif (- (aref (send tmp-cds :pos) 0) distance-thre))
              rate)
          (when (> dif 500) (setq dif 500)) ;; diff距離
          (setq dif (* dif 0.2)) ;; 上限100mm くらいに設定
          (setf (aref tv 0) 600
                (aref tv 2) (- 1112 dif))
          (when (> (abs (aref tv 1)) y-thre)
            ;; y = (raw_y / |raw_y|) * 100
            (setf (aref tv 1) (* (/ (aref tv 1) (abs (aref tv 1))) 100))
            )
          (ros::ros-info "tv:~A" tv)
;          (send (send tmp-cds :copy-worldcoords) :locate tv :world)
          (when use-arm
            (send *pr2* use-arm :inverse-kinematics (send (send tmp-cds :copy-worldcoords) :locate tv :world) :rotation-axis t)
            (send *ri* :angle-vector (send *pr2* :angle-vector) 1000 :rarm-controller)
            )))
      (when (null (send *ri* :go-waitp)) ;;t temporary
        (let ((pr2rot (elt (elt (rpy-angle (send (send *tfl* :lookup-transform "/map" "base_footprint" (ros::time 0)) :rot)) 0) 0)))
          (ros::ros-info "pr2rot:~A" pr2rot)
          (ros::ros-info "tmp-cds:~A" tmp-cds)
          (setq pr2rot (+ pr2rot 0.01))
          (cond
           ((and
             (< (abs pr2rot) 0.01)
             (< (aref (send tmp-cds :pos) 0) distance-thre)
             (or (and (null use-arm)
                      ;; temporary :rarm
                      (send *pr2* :rarm :inverse-kinematics
                            (send (send (send tmp-cds :copy-coords) :translate (float-vector 0 0 0) :world)
                                  :transform (send *pr2* :worldcoords) :world)
                            :rotation-axis t))
                 (and use-arm
                      (send *pr2* use-arm :inverse-kinematics
                            (send (send (send tmp-cds :copy-coords) :translate (float-vector 0 0 0) :world)
                                  :transform (send *pr2* :worldcoords) :world)
                            :rotation-axis t))))
            ;; it's in the region
            (when use-arm
              (send *ri* :wait-interpolation)
              (send *pr2* use-arm :inverse-kinematics
                    (send (send (send tmp-cds :copy-coords) :translate (float-vector -50 0 0) :world)
                          :transform (send *pr2* :worldcoords) :world)
                    :rotation-axis t)
              (cond
               ((equal use-arm :rarm)
                (send *ri* :angle-vector (send *pr2* :angle-vector) 1500 :rarm-controller))
               ((equal use-arm :larm)
                (send *ri* :angle-vector (send *pr2* :angle-vector) 1500 :larm-controller))
               (t
                )))
            (return-from approach-fridge-door t)
            )
           (t
            ;; out of region
            (when move
              (let ((move-dis (send (send cds :copy-worldcoords) :translate pos-offset :world))
                    )
                (ros::ros-info "x:~A y:~A pr2rot:~A"
                               (* ratio (* (aref (send move-dis :pos) 0) 0.001))
                               (* ratio (* (aref (send move-dis :pos) 1) 0.001))
                               (* ratio (- pr2rot)))
                (when move
                  (send *ri* :go-pos-unsafe-no-wait
                        (* ratio (aref (send move-dis :pos) 0) 0.001)
                        (* ratio (aref (send move-dis :pos) 1) 0.001)
                        (* 80 (- pr2rot))) ;; magic number 80
                  ))))))))))
      ;; (cond
      ;;  ((send *ri* :go-waitp) ;; moving (change arm position)
      ;;   (let ((tv (send tmp-cds :pos))
      ;;         (dif (- (aref (send tmp-cds :pos) 0) distance-thre))
      ;;         rate)
      ;;     (when (> dif 500) (setq dif 500)) ;; diff距離
      ;;     (setq dif (* dif 0.2)) ;; 上限100mm くらいに設定
      ;;     (setf (aref tv 0) 600
      ;;           (aref tv 2) (- 1112 dif))
      ;;     (when (> (abs (aref tv 1)) y-thre)
      ;;       ;; y = (raw_y / |raw_y|) * 100
      ;;       (setf (aref tv 1) (* (/ (aref tv 1) (abs (aref tv 1))) 100))
      ;;       )
      ;;     (ros::ros-info "tv:~A" tv)
      ;;     (send tmp-cds :locate tv :world)
      ;;     (when use-arm
      ;;       (send *pr2* use-arm :inverse-kinematics tmp-cds :rotation-axis t)
      ;;       (send *ri* :angle-vector (send *pr2* :angle-vector) 1000 :rarm-controller)
      ;;     )))
      ;;  (t ;; not moving
      ;;   (let ((pr2rot (elt (elt (rpy-angle (send (send *tfl* :lookup-transform "/map" "base_footprint" (ros::time 0)) :rot)) 0) 0)))
      ;;     (ros::ros-info "pr2rot:~A" pr2rot)
      ;;     (setq pr2rot (+ pr2rot 0.01))
      ;;     (cond
      ;;      ;; ik solved. Return from approach t
      ;;      ((and
      ;;        (< (abs pr2rot) 0.01)
      ;;        (< (aref (send tmp-cds :pos) 0) distance-thre)
      ;;        (or (null use-arm)
      ;;            (and use-arm
      ;;                 (send *pr2* use-arm :inverse-kinematics
      ;;                       (send (send (send tmp-cds :copy-coords) :translate (float-vector 0 0 0) :world)
      ;;                             :transform (send *pr2* :worldcoords) :world)
      ;;                       :rotation-axis t))))
      ;;       (ros::ros-info "approach-fridge-door-finish")
      ;;       ;; When already in the grasp region, just send arm to the backside of the target
      ;;       (when use-arm
      ;;         (send *pr2* use-arm :inverse-kinematics
      ;;               (send (send (send tmp-cds :copy-coords) :translate (float-vector -50 0 0) :world)
      ;;                     :transform (send *pr2* :worldcoords) :world)
      ;;               :rotation-axis t)
      ;;         )
      ;;       (warning-message 5 "not moving send angle tmp-cds:~A~%" tmp-cds)
      ;;       (cond
      ;;        ((equal use-arm :rarm)
      ;;         (send *ri* :angle-vector (send *pr2* :angle-vector) 1500 :rarm-controller))
      ;;        ((equal use-arm :larm)
      ;;         (send *ri* :angle-vector (send *pr2* :angle-vector) 1500 :larm-controller))
      ;;        (t
      ;;         ))
      ;;       (send *ri* :wait-interpolation)
      ;;       (return-from approach-fridge-door t)
      ;;       )
      ;;      (t ;; ik not solved. Fix position.
      ;;       (let ((move-dis (send (send cds :copy-worldcoords) :translate pos-offset :world))
      ;;             )
      ;;         (ros::ros-info "x:~A y:~A pr2rot:~A"
      ;;                        (* ratio (* (aref (send move-dis :pos) 0) 0.001))
      ;;                        (* ratio (* (aref (send move-dis :pos) 1) 0.001))
      ;;                        (* ratio (- pr2rot)))
      ;;         (when move
      ;;           (send *ri* :go-pos-unsafe-no-wait
      ;;                 (* ratio (aref (send move-dis :pos) 0) 0.001)
      ;;                 (* ratio (aref (send move-dis :pos) 1) 0.001)
      ;;                 (* 80 (- pr2rot))) ;; magic number 80
      ;;           ))))))))))

(defun bring-the-hand-to-the-fridge ()
  (approach-fridge-door :move nil)
  )

(defun grasp-fridge-door (&optional (cds nil) &key (door-type :circle) (move-robot t) (grasp-check t))
  (cond
   (cds
    t)
   (t
    (cond
     ((not (send *ri* :simulation-modep)) ;; real mode
      (setq cds (check-detection-srv :type "fridge" :speak-name *frige-speak-str*
                                     :detection-topic "/kinect_head/rgb/ObjectDetection"
                                     )))
     (t ;; simulation mode
      (setq cds (send (make-cube 60 60 60) :translate (float-vector 777 98 1112)))
      ))))

  (setq cds
        (make-coords :pos (send cds :worldpos)
                     :rpy (list (caar (send cds :rpy-angle)) 0 0)))
  (case door-type
    (:slide1
     (send cds :translate (float-vector 0 -230 -355)))
    (:slide2
     (send cds :translate (float-vector 0 -230 -675))))
  (send cds :transform (send *pr2* :worldcoords) :world)
  ;; temporary commentout
  ;; (let ((prev-av
  ;;        (float-vector 130.072 22.9146 57.2941 -0.001244 -117.452 0.00187 -5.72958 -360.0 -2.05726 -16.4737 -77.9557 -76.7 -162.904 -77.8544 254.584 23.0407 18.474)))
  ;;   (send *pr2* :angle-vector prev-av))

  ;; add fix *pr2*
  (let ((prev-av (send *pr2* :angle-vector))
        (hand-av))
    (send *pr2* :reset-pose)
    (setq hand-av (send *pr2* :rarm :angle-vector))
    (send *pr2* :angle-vector prev-av)
    (send *pr2* :torso :waist-z :joint-angle 130) ;; torso 動かないように
    (send *pr2* :rarm :angle-vector hand-av))
  (cond
   (;;ここもmove-armを使わない
    ;; 動きが変なのでapproach-fridge-doorのikの解き方に合わせる
    (send *pr2* :rarm :inverse-kinematics
          (send (send (send cds :copy-coords) :translate (float-vector -10 0 0) :world)
                :transform (send *pr2* :worldcoords) :world)
          :rotation-axis t)
    ;; (send *pr2* :inverse-kinematics
    ;;       (send (send cds :copy-worldcoords) :translate (float-vector 0 0 0) :local)
    ;;       :move-arm :rarm :rotation-axis t :use-torso nil :revert-if-fail nil
    ;;       :debug-view nil)

    (send *ri* :angle-vector (send *pr2* :angle-vector) 1000 :rarm-controller)
    (send *ri* :wait-interpolation)
    ;; (move-arm :rarm (send (send cds :copy-worldcoords) :translate (float-vector 25 0 0) :local)
    ;;           :use-torso nil
    ;;           :rotation-axis t :sec 1000 :move-robot t) ;; 1000必要か
    t)
   (t
    (ros::ros-warn "Could not solve ik, return from approach-fridge-door")
    (return-from grasp-fridge-door nil)))
  (let ((grasp-ret (if move-robot
                       ;; grasp直後にik解くためにこうした
                       (send *ri* :start-grasp :rarm :gain 0.1 :objects nil) 10)))
                       ;; (send *ri* :move-gripper :rarm 0.0 :effort (* 2000 0.1) :wait nil) 10)))
    ;; とりあえずcomment out   solve-ikの後にチェックは入るべき
    ;; (when (and (not (send *ri* :simulation-modep)) ;;return only in real mode
    ;;            (and grasp-check
    ;;                 (< grasp-ret 8))) ;; grasp
    ;;   (ros::ros-warn "Grasp handle failed, return from grasp-fridge-door")
    ;;   (return-from grasp-fridge-door nil))
    (return-from grasp-fridge-door t)))

(defun solve-ik-and-open-fridge-door (&optional (cds nil) &key (door-type :circle))
  (cond
   (cds
    t)
   (t
    (cond
     ((not (send *ri* :simulation-modep)) ;; real mode
      (setq cds (check-detection-srv :type "fridge" :speak-name *frige-speak-str*
                                     :detection-topic "/kinect_head/rgb/ObjectDetection"
                                     )))
     (t ;; simulation mode
      (setq cds (send (make-cube 60 60 60) :translate (float-vector 777 98 1112)))
      ))))
  (setq cds
        (make-coords :pos (send cds :worldpos)
                     :rpy (list (caar (send cds :rpy-angle)) 0 0)))
  (case door-type
    (:slide1
     (send cds :translate (float-vector 0 -230 -355)))
    (:slide2
     (send cds :translate (float-vector 0 -230 -675))))
  (send cds :transform (send *pr2* :worldcoords) :world)
  (let ((prev-av
         (float-vector 130.072 22.9146 57.2941 -0.001244 -117.452 0.00187 -5.72958 -360.0 3.53847 -14.7482 -76.7797 -53.201 -164.906 -61.8327 248.035 23.0347 18.336)))
    (send *pr2* :angle-vector prev-av))
  (warning-message 4 "[~A] aa" (ros::time-now))
  (open-fridge-traj :rarm cds 1.7 :rotation-axis t :radius 490 :wait-time t)
  )

(defun move-forward-larm (&key (ratio 1) (wait t))
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  (send *pr2* :larm :angle-vector (float-vector 20.0 45.0 53.0 -100.0 -43.0 -60.0 0.0))
  (send *ri* :angle-vector (send *pr2* :angle-vector) (* ratio 700))
  (if wait
      (send *ri* :wait-interpolation))
  )

(defun detach-fridge-handle (&key (wait t))
  (send *ri* :stop-grasp :rarm)
  (if wait
      (progn
        (send *ri* :wait-interpolation)
        ;; (send *ri* :ros-wait 0.0 :spin-self t :spin t) ;; attention-check ...
        (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
        ))
  )

(defun swipe-fridge-door (&key (door-type :circle) (ratio 1) (use-arm :larm) (wait t))
  (case door-type
    (:circle
     ;; free door handle
     (setq *free-door-pose* (send *pr2* :angle-vector))
     (send *ri* :angle-vector-sequence
           (list 
            ;; changed head-pitch to 14
            (float-vector 130 -8.34106 16.3632 56.7586 -50.7209 -42.9818 -69.7732 -6.52931 -61.9676 12.5409 -37.3903 -42.7413 338.286 -107.454 135.477 0.027956 14)
            (float-vector 130 -20.2546 15.9172 53.2397 -38.2041 -43.0083 -60.0086 -0.000466 -66.8745 25.0654 -42.6273 -49.3771 319.162 -71.6589 167.523 -0.020044 14)
            (float-vector 130 -25.9215 15.5489 55.3988 -36.512 -44.1484 -28.2568 -7.75581 -50.5861 44.487 -45.2274 -70.2799 245.195 -26.7547 148.858 0.027956 14)
            (float-vector 130 38.3106 20.1389 77.7708 -88.0224 -47.5755 -5.72958 -10.483 -12.7603 46.1883 -29.4706 -70.2965 245.815 -44.8729 150.788 0.003956 14)
            )
           (list 1200 600 1000 700))
     ;; (send *ri* :stop-grasp :rarm)
     (send *ri* :wait-interpolation)
     )
    ((:slide1 :slide2)
     (let ((rend (send *pr2* :rarm :end-coords :copy-worldcoords))
           avs)
       ;; should be changed
       (send rend :translate (float-vector -40 -160 0) *pr2*)
       (send *pr2* :inverse-kinematics rend
             :move-arm :rarm :revert-if-fail nil :use-torso 0.003)
       (push (send *pr2* :angle-vector) avs)
       (send rend :translate (float-vector 0 0 100) *pr2*)
       (send *pr2* :inverse-kinematics rend
             :move-arm :rarm :revert-if-fail nil :use-torso 0.002)
       (push (send *pr2* :angle-vector) avs)
       (send *ri* :angle-vector-sequence (nreverse avs) (list (* ratio 800) (* ratio 800)))
       (send *ri* :wait-interpolation)
       (send *pr2* :rarm :angle-vector #f(-45.0 55.0 -15.0 -120.0 12.0 -38.0 74.0))
       (send *ri* :angle-vector (send *pr2* :angle-vector) (* ratio 1600))
       ))
    ) ;; / (case door-type
  )

(defun grasp-can-init (&key (door-type :circle) (ratio 1) (use-arm :larm) (wait t))
  (case door-type
        (:circle
         (case use-arm
               (:rarm
                ;; grasp rarm
                (send *ri* :angle-vector (float-vector 130 5.30455 69.0 105.231 -88.5188 -69.9972 -5.72958 19.9717 31.3839 25.5029 23.0531 -118.916 160.305 -84.1469 160.058 -20 24) (* ratio 2000))
                ;; (let ((cds (send *pr2* :rarm :end-coords :copy-worldcoords)))
                ;;   ;; pre grasp pose
                ;;   (send cds :rotate (- (deg2rad 20)) :z)
                ;;   (send *pr2* :rarm :inverse-kinematics cds))
                ;; (send *ri* :angle-vector (send *pr2* :angle-vector) 1400)
                )
               (:larm
                ;; grasp larm
                (send *ri* :angle-vector
                      (float-vector 129.987 -5.73382 -4.97448 40.9206 -111.015 173.852 -114.592 141.026 -22.3976 72.8019 -126.402 -81.3285 -27.2228 -5.72958 177.063 -7.6493 13.038)
                      ;; (float-vector 129.977 5.4814 23.7387 42.2712 -120.629 184.315 -97.614 151.63 -22.2931 72.9182 -126.402 -81.3285 -27.2228 -5.72958 177.133 -7.6673 13.74)
                      ;; (float-vector 130.019 8.70678 19.5025 30.3731 -102.48 181.816 -83.3373 151.443 -27.9934 73.7083 -126.292 -81.4446 -27.2195 -5.72958 177.248 -7.6673 14.388)
                      (* ratio 2000))
                ;; (send *ri* :angle-vector (float-vector 130 -32.3186 26.4366 -19.6876 -118.217 -138.147 -78.3509 -166.767 -5.30455 69.0 -105.231 -88.5188 69.9972 -5.72958 -19.9717 20.0 24.0) (* ratio 2000))
                ;; (send *ri* :angle-vector-sequence
                ;;       (list (float-vector 150.0 5.0 74.0 70.0 -75.0 -70.0 -6.0 20.0 -20.0 20.0 -34.0 -110.0 12.0 -38.0 74.0 -2.0 31.0)
                ;;             ;;(float-vector 150.0 25.0 54.0 20.0 -120.0 -70.0 -6.0 20.0 -12.0 46.0 -79.0 -45.0 40.0 -22.0 27.0 -2.0 31.0)
                ;;             (float-vector 150.0 25.0 54.0 20.0 -120.0 -70.0 -6.0 20.0 -22.0 66.0 -79.0 -65.0 -40.0 -72.0 27.0 -2.0 31.0)
                ;;             (send *pr2* :angle-vector))
                ;;       (list 1400 1200 800))
                ;;         (send *ri* :start-grasp :rarm)
                ))
         (send *ri* :stop-grasp use-arm)
         (if wait (send *ri* :wait-interpolation))
         )
        ((:slide1 :slide2)
         ;; do nothing
         )
        ))

(warn ";; define open-fridge-door")
(defvar *fridge-distance-threshold* 25.0)
(defvar *fridge-rotation-threshold* 0.09)
(defun open-fridge-door (&key (open-fridge-func #'open-fridge-traj)
                              (torso-lift 130) (head-pitch 14)
                              (use-arm :larm) (move t)
                              (door-type :circle) (look-around nil) ;; :circle, :slide1, :slide2
                              (func-time 80)
                              (wait t)
                              )
  (let ((diffcds (make-coords :pos (float-vector 10000 10000 10000)))
        cds
        ret
        (ratio (/ (- func-time 56) 24.0))) ;;about 1  if ratio is smaller than 1, move speed goes up.
    ;; initial pose
    (open-fridge-door-initial-pose :door-type door-type :head-pitch head-pitch :torso-lift torso-lift :ratio ratio :wait wait)

    ;; detect fridge and open door
    (dotimes (i 10 nil)
      ;; detect fridge and move to the fridge door
      (setq cds (approach-fridge-door :door-type door-type :ratio ratio :look-around look-around :head-pitch head-pitch :torso-lift torso-lift :move move :open-fridge-func open-fridge-func :wait wait))
      (if cds (return))
      )

    (when (not cds)
      (ros::ros-info "TIMEOUT: open-fridge-door");;
      (if (and (boundp '*use-voicetext*) *use-voicetext*)
          (cond
           ((and (boundp '*use-english*) *use-english*)
            (speak-jp "I could not find the refrigerator."))
           (t (speak-jp "冷蔵庫を見つけられませんでした。")))
        (speak-jp "れいぞうこ を みつけられません でした"))
      (send *ri* :ros-wait 2.0 :spin-self t :spin t) ;; attention-check ...
      (return-from open-fridge-door nil))

    (grasp-fridge-door cds)
    (solve-ik-and-open-fridge-door cds)

    ;;move larm forward to keep the fridge door open
    (move-forward-larm :ratio ratio :wait wait)

    ;;detach fridge handle
    (detach-fridge-handle :wait wait)

    ;;swipe fridge door by larm
    (swipe-fridge-door :door-type door-type :ratio ratio :use-arm use-arm :wait wait)

    ;; grasp-can-init merge
    ;;grasp can posing
    ;;    (grasp-can-posing :door-type door-type :ratio ratio :use-arm use-arm :wait wait)

    (send *ri* :ros-wait 0.0 :spin-self t) ;; attention-check ...
    (send *ri* :wait-interpolation)
    t
    ))

(defun move-to-can-spot (&key (use-arm :rarm) (rotation 20) (move t) (pre-move t))
  (when (and move pre-move)
    (send *ri* :ros-wait 0.0 :spin-self t :spin t) ;; attention-check ...
    (ros::ros-info "GO POS UNSAFE for grasping can")
    (case use-arm
      (:rarm
       (send *ri* :go-pos-unsafe 0 0 rotation) ;; g-rarm
       (send *ri* :go-pos-unsafe 0.185 -0.25 0) ;; g-rarm
       )
      (:larm
       (send *ri* :go-pos-unsafe 0 0 rotation) ;; g-rarm
       (send *ri* :go-pos-unsafe 0.33 -0.05 0) ;; g-rarm
       ))
    (send *ri* :wait-interpolation))

  (send *ri* :ros-wait 0.0 :spin-self t :spin t) ;; attention-check ...
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  )


;; grasp-can-init merge
;; (defun grasp-can-init ()
;;   (send *pr2* :head :neck-p :joint-angle 20)
;;   (send *pr2* :head :neck-y :joint-angle 23)
;;   (send *ri* :angle-vector (send *pr2* :angle-vector) 2000 :head-controller)
;;   )

(defun approach-can (&key (move t) (use-arm :larm) (timeout 10)) ;; approach-to-can
  ;; make sure torso joint-angle is 130! If not, PR2 will grasp under point
  (let (tmp-cds
        (pos-offset (float-vector -750 0 0))
        (distance-thre 800) ;; 900 -> 800
        (y-thre 100) ;; 追っかけ手の動き幅(y)
        (default-cds (float-vector 600 0 1160))
        (end-time (ros::time+ (ros::time-now) (ros::time timeout)))
        (ratio 0.8) ;; 6-> 8
        )
    (while t
      (when (ros::time> (ros::time-now) end-time) ;; timeout
        (return-from approach-can nil))
      ;; detection
      (setq cds (check-detection-srv
                 :type "georgia"
                 :detection-topic "/kinect_head_c2/depth_registered/ObjectDetection"))
      (setq tmp-cds (send (send (send *pr2* :worldcoords) :transformation
                                (send cds :worldcoords)) :copy-worldcoords))
      (send tmp-cds :orient 0 :z :world)
      (setq *cds* tmp-cds)
      (warning-message 2 "go-waitp: ~A~%" (send *ri* :go-waitp))
      ;; for inverse-kinematics initial pose
      (send *pr2* :reset-pose)
      (send *pr2* :torso :waist-z :joint-angle 130)
#|
this version is temporary
because  :go-waitp isn't working well currently.
      when use-arm
        send angle to arm
      if moving
        nil
      if not moving
        when in region and ik solved
          when use-arm
            wait interpolation
            send angle to arm (to back front)
          return from func
        when move
          go-pos

let's go. temporary function below
|#
      (when use-arm
        (let ((tv (send (send tmp-cds :copy-worldcoords) :pos))
              (dif (- (aref (send tmp-cds :pos) 0) distance-thre))
              rate)
          (when (> dif 500) (setq dif 500)) ;; diff距離
          (setq dif (* dif 0.2)) ;; 上限10mm くらいに設定
          (setf (aref tv 0) 600
                (aref tv 2) (- 1160 dif))
          (when (> (abs (aref tv 1)) y-thre)
            ;; y = (raw_y / |raw_y|) * 100
            (setf (aref tv 1) (* (/ (aref tv 1) (abs (aref tv 1))) 100))
            )
          (ros::ros-info "tv:~A" tv)
          ;; (send tmp-cds :locate tv :world)
          (when use-arm
            (send *pr2* use-arm :inverse-kinematics (send (send tmp-cds :copy-worldcoords) :locate tv :world) :rotation-axis t)
            (send *ri* :angle-vector (send *pr2* :angle-vector) 1000 :larm-controller)
            )))
      (when (null (send *ri* :go-waitp))
        (let ((pr2rot (elt (elt (rpy-angle (send (send *tfl* :lookup-transform "/map" "base_footprint" (ros::time 0)) :rot)) 0) 0)))
          (ros::ros-info "pr2rot:~A" pr2rot)
        (cond
         ((and
           (< (abs pr2rot) 0.01)
           (< (aref (send tmp-cds :pos) 0) distance-thre)
           (or (and (null use-arm)
                    ;; temporary larm
                    (send *pr2* :larm :inverse-kinematics
                          (send (send (send tmp-cds :copy-coords) :translate (float-vector 0 0 0) :world)
                                :transform (send *pr2* :worldcoords) :world)
                          :rotation-axis t))
               (and use-arm
                    (send *pr2* use-arm :inverse-kinematics
                          (send (send (send tmp-cds :copy-coords) :translate (float-vector 0 0 0) :world)
                                :transform (send *pr2* :worldcoords) :world)
                          :rotation-axis t))))
          (ros::ros-info "cds:~A" tmp-cds)
          (ros::ros-info "end approach-can")
          (when use-arm
            (send *ri* :wait-interpolation)
            (send *pr2* use-arm :inverse-kinematics
                  (send (send (send tmp-cds :copy-coords) :translate (float-vector -100 0 0) :world)
                        :transform (send *pr2* :worldcoords) :world)
                  :rotation-axis t)
            (cond
             ((equal use-arm :rarm)
              (send *ri* :angle-vector (send *pr2* :angle-vector) 1500 :rarm-controller))
             ((equal use-arm :larm)
              (send *ri* :angle-vector (send *pr2* :angle-vector) 1500 :larm-controller))
             (t
              )))
          (return-from approach-can t)
          )
         (t
          ;; not in the region
          (when move
            (let ((move-dis (send (send cds :copy-worldcoords) :translate pos-offset :world))
                  )
              (ros::ros-info "x:~A y:~A pr2rot:~A"
                             (* ratio (* (aref (send move-dis :pos) 0) 0.001))
                             (* ratio (* (aref (send move-dis :pos) 1) 0.001))
                             (* ratio (- pr2rot)))
              (when move
                (send *ri* :go-pos-unsafe-no-wait
                      (* ratio (aref (send move-dis :pos) 0) 0.001)
                      (* ratio (aref (send move-dis :pos) 1) 0.001)
                      (* 80 (- pr2rot))) ;; magic number 80
                ))))))))))

      ;; (cond
      ;;  ((send *ri* :go-waitp) ;; moving (change arm position)
      ;;   ;; (setq tmp-cds
      ;;   ;;       (send (send (send tmp-cds :copy-coords) :translate (float-vector -200 0 -100) :world)
      ;;   ;;             :transform (send *pr2* :worldcoords) :world))
      ;;   ;; (unless (send *pr2* use-arm :inverse-kinematics tmp-cds :rotation-axis t)
      ;;   ;;   (send tmp-cds :locate default-cds :world)
      ;;   ;;   (send *pr2* use-arm :inverse-kinematics tmp-cds  :rotation-axis t))
      ;;   ;; (warning-message 5 "moving send angle~%")
      ;;   ;; (cond
      ;;   ;;  ((equal use-arm :rarm)
      ;;   ;;   (send *ri* :angle-vector (send *pr2* :angle-vector) 1000 :rarm-controller))
      ;;   ;;  (t
      ;;   ;;   (send *ri* :angle-vector (send *pr2* :angle-vector) 1000 :larm-controller)))
      ;;   ;; nil
      ;;   (let ((tv (send tmp-cds :pos))
      ;;         (dif (- (aref (send tmp-cds :pos) 0) distance-thre))
      ;;         rate)
      ;;     (when (> dif 500) (setq dif 500)) ;; diff距離
      ;;     (setq dif (* dif 0.2)) ;; 上限10mm くらいに設定
      ;;     (setf (aref tv 0) 600
      ;;           (aref tv 2) (- 1160 dif))
      ;;     (when (> (abs (aref tv 1)) y-thre)
      ;;       ;; y = (raw_y / |raw_y|) * 100
      ;;       (setf (aref tv 1) (* (/ (aref tv 1) (abs (aref tv 1))) 100))
      ;;       )
      ;;     (ros::ros-info "tv:~A" tv)
      ;;     (send tmp-cds :locate tv :world)
      ;;     (when use-arm
      ;;       (send *pr2* use-arm :inverse-kinematics tmp-cds :rotation-axis t)
      ;;       (send *ri* :angle-vector (send *pr2* :angle-vector) 1000 :larm-controller)
      ;;     )))
      ;;  (t ;; not moving
      ;;   (let ((pr2rot (elt (elt (rpy-angle (send (send *tfl* :lookup-transform "/map" "base_footprint" (ros::time 0)) :rot)) 0) 0)))
      ;;     (ros::ros-info "pr2rot:~A" pr2rot)
      ;;     (cond
      ;;      ;; ik solved. Return from approach t
      ;;      ((and
      ;;        (< (abs pr2rot) 0.01)
      ;;        (< (aref (send tmp-cds :pos) 0) distance-thre)
      ;;        (or (null use-arm)
      ;;            (and use-arm
      ;;                 (send *pr2* use-arm :inverse-kinematics
      ;;                       (send (send (send tmp-cds :copy-coords) :translate (float-vector 0 0 0) :world)
      ;;                             :transform (send *pr2* :worldcoords) :world)
      ;;                       :rotation-axis t))))
      ;;       (ros::ros-info "approach-can-finish")
      ;;       ;; When already in the grasp region, just send arm to the backside of the target
      ;;       (when use-arm
      ;;         (send *pr2* use-arm :inverse-kinematics
      ;;               (send (send (send tmp-cds :copy-coords) :translate (float-vector -100 0 0) :world)
      ;;                     :transform (send *pr2* :worldcoords) :world)
      ;;               :rotation-axis t)
      ;;         )
      ;;       (warning-message 5 "not moving send angle tmp-cds:~A~%" tmp-cds)
      ;;       (cond
      ;;        ((equal use-arm :rarm)
      ;;         (send *ri* :angle-vector (send *pr2* :angle-vector) 1500 :rarm-controller))
      ;;        ((equal use-arm :larm)
      ;;         (send *ri* :angle-vector (send *pr2* :angle-vector) 1500 :larm-controller))
      ;;        (t
      ;;         ))
      ;;       (return-from approach-can t)
      ;;       )
      ;;      (t ;; ik not solved. Fix position.
      ;;       (let ((move-dis (send (send cds :copy-worldcoords) :translate pos-offset :world))
      ;;             )
      ;;         (ros::ros-info "x:~A y:~A pr2rot:~A"
      ;;                        (* ratio (* (aref (send move-dis :pos) 0) 0.001))
      ;;                        (* ratio (* (aref (send move-dis :pos) 1) 0.001))
      ;;                        (* ratio (- pr2rot)))
      ;;         (when move
      ;;           (send *ri* :go-pos-unsafe-no-wait
      ;;                 (* ratio (aref (send move-dis :pos) 0) 0.001)
      ;;                 (* ratio (aref (send move-dis :pos) 1) 0.001)
      ;;                 (* 80 (- pr2rot))) ;; magic number 80
      ;;           ))))))))))

(defun bring-the-hand-to-the-can ()
  (approach-can :move nil)
  )

(defun catch-can (&key (use-arm :larm))
  (let (tmp-cds)
    (setq cds (check-detection-srv
               :type "georgia"
               :detection-topic "/kinect_head_c2/depth_registered/ObjectDetection"))
    (setq tmp-cds (send (send (send *pr2* :worldcoords) :transformation
                              (send cds :worldcoords)) :copy-worldcoords))
    (send tmp-cds :orient 0 :z :world)
    ;; for inverse-kinematics initial pose
    (send *pr2* :reset-pose)
    (send *pr2* :torso :waist-z :joint-angle 130)

    (send *pr2* use-arm :inverse-kinematics
          (send (send (send tmp-cds :copy-coords) :translate (float-vector 40 0 0) :world)
                :transform (send *pr2* :worldcoords) :world)
          :rotation-axis t)
    (cond
     ((equal use-arm :rarm)
      (send *ri* :angle-vector (send *pr2* :angle-vector) 1500 :rarm-controller))
     (t
      (send *ri* :angle-vector (send *pr2* :angle-vector) 1500 :larm-controller)))
    (send *ri* :wait-interpolation)
    (send *ri* :start-grasp use-arm)
    ))

(defun put-arm-to-the-outside (&key (use-arm :larm))
  ;; (let (av
  ;;       (default-cds (send (make-coords) :translate (float-vector 700 0 1200) :world))
  ;;       )
  ;;   ;; for inverse-kinematics initial pose
  ;;   (send *pr2* :reset-pose)
  ;;   (send *pr2* :torso :waist-z :joint-angle 130)
  ;;   (setq av (send *pr2* use-arm :inverse-kinematics default-cds :rotation-axis t))
  ;;   (cond
  ;;    ((eq use-arm :rarm)
  ;;     (send *ri* :angle-vector av 3000 :rarm-controller))
  ;;    (t
  ;;     (send *ri* :angle-vector av 3000 :larm-controller)))))
  (approach-can) ;; temporary
  )

(defparameter *can-cds* nil)
(defun grasp-can-motion (&key (use-arm :rarm) (rotation 20))
  ;; detect cans which was indicated by ( type )
  ;;(when (not (setq cds (check-detection :type *type* :single t)))
  ;;(setq cds (check-detection :type *type* :tf-force t :timeout 30 :single t)))
  (send *ri* :stop-grasp use-arm)
  (let (cds isgrasp)
    (cond
     ((not (send *ri* :simulation-modep)) ;; real mode
      (unless (setq cds (check-detection :speak-name *type*
                                         :type *type* :tf-force t :timeout 35 :single t
                                         :detection-topic "/kinect_head_c2/depth_registered/ObjectDetection"))
        (setq cds (check-detection :speak-name *type*
                                   :type *type* :tf-force t :timeout 70 :single t
                                   :detection-topic "/kinect_head_c2/depth_registered/ObjectDetection"))))
     (t ;; simulation mode
      (setq cds (send (make-cube 60 60 60) :translate (float-vector 756 302 1164)))))
    (setq *can-cds* cds)
    (send *ri* :ros-wait 0.0 :spin-self t :spin t) ;; attention-check ...
    (when cds
      ;; (speak-jp (format nil "~A を とりだします" *type*))
      (send *obj* :reset-coords)
      (send *obj* :transform cds)
      (warn ";; print check cans~%")
      (dump-structure *standard-output*
                      (list (send *pr2* :copy-worldcoords)
                            (send cds :copy-worldcoords)))
      (unless (boundp '*tfb*)
        (setq *tfb* (instance ros::transform-broadcaster :init)))
      (send *tfb* :send-transform
            cds "/base_footprint" "/object_pose_frame")
      (send *ri* :ros-wait 0.0 :spin-self t :spin t) ;; attention-check ...
      (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
      (send cds :transform (send *pr2* :worldcoords) :world)
      (setq *last-can-coords* cds)
      (grasp-can-single cds :rotation rotation :use-arm use-arm)
      (setq isgrasp (< 10 (send *ri* :start-grasp use-arm :objects nil)))
      (if (send *ri* :simulation-modep) ;; in simulation mode, set isgrasp true
          (setq isgrasp t))
      (ros::ros-info "isgrasp : ~A" isgrasp)
      (when isgrasp
        (return-from grasp-can-motion t))
      ;; (unix::sleep 2)
      (if (and (boundp '*use-voicetext*) *use-voicetext*)
          (cond
           ((and (boundp '*use-english*) *use-english*)
            (speak-jp "I failed to pick up. I will retry."))
           (t (speak-jp "失敗しましたもう一度やってみます。")))
        (speak-jp "しっぱいしました  もういちど やって みます"));;
      ) ;; / (when cds
    (return-from grasp-can-motion isgrasp)
    ))

(defun go-back-from-fridge (&key (isgrasp t) (move t) (use-arm :larm) (rotation 20))
  (when isgrasp
    (when move
      (case use-arm
        (:rarm
         (send *ri* :go-pos-unsafe -0.3 0.05 (- rotation)))
        (:larm
         (send *ri* :go-pos-unsafe -0.385 -0.205 1))))
         ;; (send *ri* :go-pos-unsafe -0.385 -0.02 -4)))) ;;yesterday
         ;; (send *ri* :go-pos-unsafe -0.3 -0.2 (- rotation)))))
    (send *ri* :ros-wait 0.0 :spin-self t :spin t) ;; attention-check ...
    (return-from go-back-from-fridge t))
  ;; (unix::sleep 2)
  (if (and (boundp '*use-voicetext*) *use-voicetext*)
      (cond
       ((and (boundp '*use-english*) *use-english*)
        (speak-jp (format nil "I could no pick up ~A" *type*)))
       (t (speak-jp (format nil "~Aを取り出しませんでした。" *type*))))
    (speak-jp (format nil "~A を とりだし ません でした" *type*)))
  nil
  )

(warn ";; define grasp-can")
(defvar *use-arm-navigation* nil)
(defun grasp-can (&key (move t) (pre-move t) (post-move t) (move-only nil)
                       (rotation 20) (use-arm :larm) (wait t))
  (if (eq use-arm :larm) (setq rotation (- rotation)))
  ;; move-to can spot for grasping


  ;; (move-to-can-spot :use-arm use-arm :rotation rotation :move move :pre-move pre-move)
  (grasp-can-init)
  (approach-can :use-arm :larm)
  (catch-can :use-arm :larm)
  (put-arm-to-the-outside :use-arm :larm)
  (go-back-from-fridge :isgrasp isgrasp :move move :use-arm use-arm :rotation rotation)


  ;; (when (not move-only)
  ;;   (let (isgrasp res)
  ;;     ;; grasp can
  ;;     (unix:sleep 2)
  ;;     (dotimes (trial 10)
  ;;       (setq isgrasp (grasp-can-motion :use-arm use-arm :rotation rotation))
  ;;       (if isgrasp (return))
  ;;       )
  ;;     (setq res (go-back-from-fridge :isgrasp isgrasp :move move :use-arm use-arm :rotation rotation))
  ;;     (when res (return-from grasp-can t))
  ;;     )
  ;;   )
  ;; (when (and move post-move)
  ;;   (if (eq use-arm :larm) (setq rotation (- rotation)))
  ;;   (case use-arm
  ;;     (:rarm
  ;;      (send *ri* :go-pos-unsafe -0.3 0.05 (- rotation)))
  ;;     (:larm
  ;;      (send *ri* :go-pos-unsafe -0.3 -0.2 (- rotation)))))
  ;; (send *ri* :ros-wait 1.0 :spin-self t :spin t) ;; attention-check ...
  ;; nil)
  )

(warn ";; define close-fridge")
(defun close-fridge (&key (use-arm :larm) (harf nil) (wait t))
  ;; close the door of fridge
  (send *ri* :ros-wait 0.0 :spin-self t) ;; attention-check ...

  (case use-arm
    (:rarm
     (send *ri* :angle-vector-sequence
           (list (float-vector 130 5 74 100 -90 -70 -6 20 -13 50 -30 -25 118 -90 62 -2 31);; 2000
                 (float-vector 130 5 74 100 -90 -70 -6 20 -50 54 -15 -43 118 -90 62 -2 31);; 1000
                 (float-vector 130 5 74 100 -90 -70 -6 20 -80 10 -39 -33 -49 -32 89 -2 32);; 1500
                 (float-vector 130 5 74 100 -90 -70 -6 20 -61 6 -74 -44 55 -75 102 -2 33) ;; 1500 ;; pre-close
                 ;;(float-vector 50 5 74 100 -90 -70 -6 20 9.05631 26.4211 -42.0907 -111.215 97.6198 -19.5577 -24.0422 -2.0 33.0)
                 (float-vector 75 5 74 100 -90 -70 -6 20
                               ;;-18.5 5 -70 -39 66 -48 66
                               -35 13 -71 -89 50 -53 46
                               -2.0 33.0) ;; 2400
                 (float-vector 50 5 74 100 -90 -70 -6 20
                               24 3 -2 -18 -40 -16 0
                               -2.0 33.0))  ;; 2500
           (list 2000 1000 1500 1500 2600 3000))
     )
    (:larm
     (send *ri* :angle-vector-sequence
	   (list
	    (float-vector 130 30.5203 45.755 4.94941 -109.614 -181.537 -6.86273 -181.599 -53.1607 47.972 -113.924 -85.8823 69.9861 -5.72958 -19.9642 20.098 23.796)
	    (float-vector 130 30.5203 45.755 4.94941 -109.614 -181.537 -6.86273 -181.599 -52.6619 48.6215 -58.4944 -63.1464 52.4 -18.6578 -36.158 20.068 23.844)
	    (float-vector 130 30.5203 45.755 4.94941 -109.614 -181.537 -6.86273 -181.599 -47.223 7.4904 -68.5549 -27.9932 -5.62513 -42.8014 -3.89764 20.074 23.874)
	    (float-vector 130 30.5203 45.784 5.36286 -109.622 -181.537 -8.06679 -181.631 -59.7112 14.9014 -67.1032 -63.3787 -27.4106 -22.5567 -5.52051 20.056 23.958)
	    (float-vector 130 30.4823 45.7646 5.60174 -109.556 -181.537 -8.78224 -181.609 -39.5086 18.5463 -68.6927 -77.5544 -27.1157 -22.6614 -5.46068 20.074 23.844)
	    (float-vector 130 30.4538 45.7162 5.80387 -109.514 -181.537 -9.29827 -181.561 -5.03645 8.29984 -64.2091 -40.9414 -24.0764 -6.03142 -7.88126 20.08 23.778)
	    (float-vector 130 30.5013 45.7743 5.23423 -109.572 -181.537 -7.40368 -181.626 -20.6884 41.162 -12.6847 -106.395 -0.527595 -7.67921 -0.599568 20.038 23.814))
	   (list 500 500 500 800 1000 1000 600)
	   )
     ))
  (if wait
      (progn
        (send *ri* :wait-interpolation)
        (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
        (send *ri* :ros-wait 0.0 :spin-self t) ;; attention-check ...
        ))
  ;;  (pr2-tuckarm-pose use-arm :outside)
  (send *pr2* :reset-pose)
  (send *pr2* :torso :waist-z :joint-angle 130)
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  t
  )

(defun put-can-on-turtlebot ()
  (send *ri* :update-robot-state)
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  (pr2-tuckarm-pose :rarm)
  (send *pr2* :rarm :inverse-kinematics (make-coords :pos (float-vector 800 0 730)))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 6000)
  (send *ri* :wait-interpolation)
  (send *pr2* :rarm :inverse-kinematics (make-coords :pos (float-vector 800 0 530)))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  (send *ri* :move-gripper :rarm 0.08 :wait t)
  (send *pr2* :rarm :inverse-kinematics (make-coords :pos (float-vector 650 0 530)))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  (send *ri* :move-gripper :rarm 0.005 :wait t)
  (pr2-tuckarm-pose :rarm)
  )

(defun put-can-on-turtlebot2 ()
  (send *ri* :update-robot-state)
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  (pr2-tuckarm-pose :rarm)
  (send *pr2* :rarm :inverse-kinematics (send (make-coords :pos (float-vector 800 0 730)) :rotate (deg2rad 180) :x))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 6000)
  (send *ri* :wait-interpolation)
  (send *pr2* :rarm :inverse-kinematics (send (make-coords :pos (float-vector 800 0 550)) :rotate (deg2rad 180) :x))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  (send *ri* :move-gripper :rarm 0.08 :wait t)
  (send *pr2* :rarm :inverse-kinematics (send (make-coords :pos (float-vector 650 0 530)) :rotate (deg2rad 180) :x))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  (send *ri* :move-gripper :rarm 0.005 :wait t)
  (pr2-tuckarm-pose :rarm)
  )


(defun distance-of-front-cb (msg)
  (let* ((pc (send msg :ranges))
		 (about-front (remove-if #'zerop 
								 (subseq pc (- (/ (length pc) 2) 10) (+ (/ (length pc) 2) 10)))))
	(push (/ (reduce #'+ about-front) (length about-front))
		  *averages*)
	))

(defun distance-of-front ()
  (ros::roseus "place_tray_base_scan_calc")
  (ros::roseus-add-msgs "sensor_msgs")
  (ros::roseus-add-msgs "posedetection_msgs")
  (ros::roseus-add-msgs "geometry_msgs")
  (defvar *base-scan-id* "/base_scan")
  (defparameter *averages* nil)

  (ros::subscribe *base-scan-id*
				  sensor_msgs::LaserScan
				  #'distance-of-front-cb)

  (ros::rate 50)
  (when (ros::ok)
	(dotimes (i 30)
	  (ros::spin-once)
	  (ros::sleep)
	  ))
  (ros::unsubscribe *base-scan-id*)
  (/ (reduce #'+ *averages*) (length *averages*))
  )

(warn ";; define place-tray")
(defun place-tray (&key (table-height 740.0)
                        (up-height-tolerance 100.0)
                        (down-height-tolerance -20.0)
                        (use-base-scan nil)
                        (go-forward 0)
                        (forward-dist 250.0))
    ;;
    ;; place_tray function
    ;;
    ;; Before this function is called, the robot is assumed to have the tray gripped in both grippers
    ;; and the tray is posiitoned directly above the table.
    ;;
  (let (arm-mid-pos
        arm-mid-pos-pr2)

    ;; get current gripper pose
    (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
    (if (> table-height (+ 750 (send *pr2* :torso :waist-z :joint-angle)))
        (ros::ros-info "move torso up")
        (send *pr2* :torso :waist-z :joint-angle (- table-height 750 (send *pr2* :torso :waist-z :joint-angle))))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
    (send *ri* :wait-interpolation)

    (setq arm-mid-pos
          (apply #'midpoint 0.5 (send *pr2* :arms :end-coords :worldpos)))

    (setq arm-mid-pos-pr2 (send (send *pr2* :worldcoords) :inverse-transform-vector
                                (apply #'midpoint 0.5 (send *pr2* :arms :end-coords :worldpos))))
    ;;raise tray
    (let ((av-seq nil))
      (send *pr2* :arms :move-end-pos
            (float-vector 0 0 (- (+ table-height up-height-tolerance) (elt arm-mid-pos 2))) :world)
      (push (send *pr2* :angle-vector) av-seq)

      (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
      ;; put forward a tray
;;      (send *pr2* :arms :move-end-pos
;;            (send *pr2* :rotate-vector (float-vector (- forward-dist (elt arm-mid-pos-pr2 0)) 0 0)) :world)
      (send *pr2* :arms :move-end-pos (float-vector forward-dist 0 0) *pr2*)
      (push (send *pr2* :angle-vector) av-seq)
      (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))

      (setq av-seq (reverse av-seq))
      (send *ri* :angle-vector-sequence av-seq (list 2000 2000))
      (send *ri* :wait-interpolation)
      )

    ;; for navigation
    (when use-base-scan
      (setq go-forward (* (- (distance-of-front) 0.1) 1000))
      (if (> 0.1 go-forward) (setq go-forward 0)))
    (when (> go-forward 10)
      (speak-jp "まえにすすみます")
;;      (send *pr2* :translate (float-vector go-forward 0 0))
      (send *ri* :go-pos-unsafe (/ go-forward 1000.0) 0)
      )

    ;; put down a tray
;;    (send *pr2* :arms :move-end-pos
;;          (float-vector 0 0 (- (- table-height down-height-tolerance) (elt arm-mid-pos 2))) :world)
    (setq arm-mid-pos
          (apply #'midpoint 0.5 (send *pr2* :arms :end-coords :worldpos)))
#|
    (let ((mtrq 0))
      (while (and (< (- table-height down-height-tolerance (elt arm-mid-pos 2)) 0) (< mtrq 0.4))
        (send *ri* :state)
        (setq mtrq (apply #'max (mapcar #'(lambda (j) (/ (abs (send j :joint-torque))
                                                          (send j :max-joint-torque)))
                                         (flatten (send *ri* :robot :arms :joint-list)))))
        (send *pr2* :arms :move-end-pos (float-vector 0 0 -50) :world)
        (send *ri* :angle-vector (send *pr2* :angle-vector) 500)
        (send *ri* :wait-interpolation)
        ))
|#
    (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
    (send *pr2* :arms :move-end-pos (float-vector 0 0 -200) :world)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
    (ros::rate 10)
    (let* (av
          (prev-cds #f(0 0 0))
          (diff-height (abs (- (elt prev-cds 2) (elt (apply #'midpoint 0.5 (send *pr2* :arms :end-coords :worldpos)) 2)))))
      (while (> diff-height 0.05)
        (ros::ros-info "diff height: ~A" diff-height)
        (setq prev-cds (apply #'midpoint 0.5 (send *pr2* :arms :end-coords :worldpos)))
        (send *ri* :update-robot-state)
        (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
        (unix:usleep 100000)
        (ros::spin-once)
        (ros::sleep)
        (setq diff-height (abs (- (elt prev-cds 2) (elt (apply #'midpoint 0.5 (send *pr2* :arms :end-coords :worldpos)) 2))))
        )
    (send *ri* :stop-motion)
    (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
    (ros::ros-info "diff height ~A < 10 -> open gripper" diff-height))

    ;; open grippers
    (send *ri* :stop-grasp :arms :wait t)
;;     (send *pr2* :larm :end-coords :dissoc target) ;; ???

    ;; move arms apart
    (send *pr2* :torso :waist-z :joint-angle (+ (send *pr2* :torso :waist-z :joint-angle) 100))
    (send *pr2* :arms :move-end-pos (float-vector 0 0 100) :world)
    (send *pr2* :arms :move-end-pos (float-vector 0 0 100) :world)
    (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
    (send *ri* :wait-interpolation)
    (send *pr2* :arms :move-end-pos (float-vector -250 0 -50) *pr2*)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
    (send *ri* :wait-interpolation)

    ;; for navigation
    (when (> go-forward 10)
      (speak-jp "うしろにさがります")
      (send *pr2* :translate (float-vector (- go-forward) 0 0))
      (send *ri* :go-pos-unsafe (/ go-forward -1000.0) 0)
      )

    ;; revert torso link
    (send *pr2* :torso :waist-z :joint-angle 50)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
    ;;dont wait interpolation
    (pr2-reset-pose)
    t
    ))

(warn ";; define pick-tray")
(load "models/room610-tray-object.l");; for room610-tray
(defun pick-tray (&key (target (room610-tray))
                       (manip-pose t)
                       (desired-tray-pose
                        (make-coords :pos (float-vector 600 0 800)
                                     :rpy (list -pi/2 0 0)))
                       )
  (let (tray-larm-handle tray-rarm-handle
        (grasp-threshold (list 3 3))
        (grasp-args)) ;; use let to define local variables
    (warn "pick-tray-table-side ~%")
    (defparameter *room73b2-origin* "/eng2/7f/73B2")
    (when manip-pose
      (pr2-pick-tray-pose :reset-pose t)
      (send *pr2* :head :neck-p :joint-angle 50)
      (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
      (send *ri* :wait-interpolation))

    ;;detect tray when real robot
    (when (not (send *ri* :simulation-modep))

      (let ((cnt 0))
        (while (and (<= cnt 3) (not (tray-detection target)))
          (send *pr2* :head :neck-p :joint-angle (+ 50 (* (10 (incf cnt)))))
          (send *ri* :angle-vector (send *pr2* :angle-vector) 600)
          (send *ri* :wait-interpolation)
          ))

      (dotimes (i 10)
        (let ((cds
               (check-detection :type "tray_center" ;; work for any object
                                :speak-name "tray"
                                :target-object target
                                :timeout 30
                                :diff-position 10
                                :diff-rotation (deg2rad 10)
                                :speak t))
              diffcds)
          (when cds
            (send *ri* :ros-wait 0.0 :spin-self t) ;; attention-check ...

            ;; rotate desired-tray-pose if detected tray is 180deg rotated
            (if (> (abs (rad2deg (elt (send cds :difference-rotation desired-tray-pose) 2))) 90)
                (send desired-tray-pose :rotate pi :z))

            (setq diffcds (send (send cds :copy-worldcoords)
                                :transform (send desired-tray-pose :inverse-transformation)))
            (warn "~%~A -> ~A / ~A~%" diffcds cds desired-tray-pose) 
            (ros::ros-info "DIFF: ~A" diffcds)

            (if (and (< (norm (float-vector (elt (send diffcds :worldpos) 0)
                                            (elt (send diffcds :worldpos) 1)))
                        *fridge-distance-threshold*)
                     (< (abs (elt (car (rpy-angle (send diffcds :worldrot))) 0))
                        *fridge-rotation-threshold*))
                (return))

            (send *ri* :go-pos-unsafe
                  (/ (elt (send diffcds :worldpos) 0) 1100.0)
                  (/ (elt (send diffcds :worldpos) 1) 1100.0)
                  (* 0.9 (rad2deg (elt (car (rpy-angle (send diffcds :worldrot))) 0))))

            ;; wait ???
            (send *ri* :ros-wait 0.6 :spin-self t :spin t) ;; attention-check ...
            )))
      )
    ;;
    (send target :transform  (send *pr2* :worldcoords) :world)
    ;;
    (setq tray-larm-handle
          (send (send (send target :handle-larm-handle) :copy-worldcoords)
                :rotate (deg2rad -20) :z)) ;; do not move in world coords, use object(tray) relative coords
    (setq tray-rarm-handle
          (send (send (send target :handle-rarm-handle) :copy-worldcoords)
                :rotate (deg2rad  20) :z)) ;; need copy-worldcoords, otherwise move handle coords directory

    ;; Open gripper
    (send *ri* :stop-grasp :arms) ;; stop-grasp wait until gripper motion stops
    ;; wait interpolation

    ;; use dual arm IK (see https://sourceforge.net/p/jsk-ros-pkg/code/4103/)
    (if (boundp '*irtviewer*)
        (send-all (list tray-larm-handle tray-rarm-handle) :draw-on :flush t)) ;; use :draw-on to confirm coords position

    (let (avs)
      (dolist (offset (list (cons (float-vector -100 -10 0)
                                  (float-vector -100 10 0))
                            (cons (float-vector -25 15 0)
                                  (float-vector -25 -15 0))))
        (send *pr2* :inverse-kinematics
              (list   ;; use local coords, this cod works if tray is placed on different orientation
               (send (send tray-larm-handle :copy-worldcoords) :translate (car offset))
               (send (send tray-rarm-handle :copy-worldcoords) :translate (cdr offset)))
              :use-torso t
              :look-at-target (midpoint 0.5
                                        (send tray-larm-handle :worldpos)
                                        (send tray-rarm-handle :worldpos)))
        (push (send *pr2* :angle-vector) avs)
        )
      (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
      ;;(print (send *pr2* :angle-vector))
      ;;(print (send *pr2* :arms :wrist-r :joint-angle))
      (send *ri* :angle-vector-sequence (reverse avs) (list 2000 1200))
      ;;(send *ri* :angle-vector-with-constraint (send *pr2* :angle-vector) 1000 :arms)
      (send *ri* :wait-interpolation)
      )

    (warning-message 3 "grasp tray~%")
    ;; Grasp it
    (setq grasp-args (send *ri* :start-grasp :arms :gain 1 :objects
                           (list (find target (send *ri* :objects)
                                       :test #'(lambda (a b) (string= (send a :name)
                                                                      (send b :name)))))))
    ;; stop-grasp wait until gripper motion stops

    ;; check if tray is grasped
    (warning-message 3 "check tray ~A > ~A~%" grasp-args grasp-threshold)
    (when (and (not (send *ri* :simulation-modep))
               (or (< (elt grasp-args 0) (elt grasp-threshold 0))
                   (< (elt grasp-args 1) (elt grasp-threshold 1))))
      (warning-message 3 "tray is not grasped~%")
      (speak-jp "トレイを掴めませんでした。")

      ;; Open gripper
      (send *ri* :stop-grasp :arms) ;; stop-grasp wait until gripper motion stops
      (pr2-pick-tray-pose)
      (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
      (send *ri* :wait-interpolation)
      (return-from pick-tray nil)
      )
    (unwind-protect
        (progn
;;          (send *pr2* :larm :end-coords :assoc target) ;; ???

          (let (avs)
            ;; Hold it up!
            (send *pr2* :arms :move-end-pos (float-vector 0 0 100) :world)
            (send *pr2* :look-at-hand :arms)
            (push (send *pr2* :angle-vector) avs)
            (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
            ;; Move it close to PR2
            (send *pr2* :arms :move-end-pos (float-vector -250 0 0) *pr2*)
            (send *pr2* :look-at-hand :arms)
            (push (send *pr2* :angle-vector) avs)
            (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))

            (send *ri* :angle-vector-sequence (reverse avs) 1400) ;; use angle-vector-sequence
            (send *ri* :wait-interpolation)
          ;; Go back 50cm
          ;;(send *ri* :go-pos-unsafe -0.4 0 90)
            t))
      (return-from pick-tray nil))
    ))

(ros::roseus-add-msgs "pr2_gripper_sensor_msgs")
(defun wait-for-hand-impact (arm &key (timeout 30))
  (let* ((action-name (format nil "/~c_gripper_sensor_controller/event_detector" (if (eq arm :larm) #\l #\r)))
         (client (instance ros::simple-action-client :init action-name pr2_gripper_sensor_msgs::PR2GripperEventDetectorAction))
         (goal (instance pr2_gripper_sensor_msgs::PR2GripperEventDetectorActionGoal :init)))
    (unless (send client :wait-for-server 5)
      (return-from wait-for-hand-impact nil))
    (send goal :header :stamp (ros::time-now))
    (send goal :goal_id :stamp (ros::time-now))
    ;;(send goal :goal :command :trigger_conditions pr2_gripper_sensor_msgs::PR2GripperEventDetectorCommand::*FINGER_SIDE_IMPACT_OR_SLIP_OR_ACC*)
    (send goal :goal :command :trigger_conditions pr2_gripper_sensor_msgs::PR2GripperEventDetectorCommand::*FINGER_SIDE_IMPACT_OR_ACC*)
    ;;(send goal :goal :command :trigger_conditions pr2_gripper_sensor_msgs::PR2GripperEventDetectorCommand::*SLIP*)
    (send goal :goal :command :slip_trigger_magnitude 0.02)
    (send goal :goal :command :acceleration_trigger_magnitude 3.0) ;; m/s^2
    (send client :send-goal goal)
    (ros::ros-info "wait for touching robot hand")
    (send client :wait-for-result :timeout timeout))
  )

(defun hand-over (arm &key (wait-shock nil))
  (ros::spin-once)
  (let* ((av (send *ri* :state :potentio-vector))
         (tuckarm (check-tuckarm-pose))
         (isfreearm (eq arm tuckarm))
         ;; this is for :larm
         (avs (list #f(12 0 64 70 -122 50 -115 160 -4 74 -105 -90 70 -5 20 2 15)
                    #f(12 6 9 106 -77 35 -124 -128 -4 75 -104 -89 70 0 20 3 30)
                    #f(12 13 21 62 -105 -117 -66 -71 -4 74 -104 -89 70 -5 20 4 40)
                    #f(12 9 24 50 -94 -158 -70 39 -4 74 -104 -89 70 -5 20 5 30)))
         (tms (make-list (length avs) :initial-element 1000))
         (l-r-reverse #f(1  -1 1 -1 1 -1 1 -1  -1 1 -1 1 -1 1 -1  -1 1)))
    ;;
    (if (eq arm :rarm)
        (setq avs
              (mapcar #'(lambda(av)
                          (map float-vector #'*
                               (concatenate float-vector
                                            (subseq av 0 1) (subseq av 8 15)
                                            (subseq av 1 8) (subseq av 15 17))
                               l-r-reverse))
                      avs)))
    ;;
    (unless isfreearm
      (pr2-reset-pose)
      (let ((avs-length (length avs)))
	(setq avs (subseq avs (- avs-length 2))
	      tms (subseq tms (- avs-length 2))
	      av (send *ri* :state :potentio-vector)))
      (setq tuckarm arm))
    ;;
    (send *ri* :angle-vector-sequence avs tms)
    (send *ri* :wait-interpolation)
    ;;
    (if (and wait-shock (not (numberp wait-shock))) ;; if wait-shock = t
        (setq wait-shock 10))
    (if wait-shock
        (progn (wait-for-hand-impact arm :timeout wait-shock)
               (ros::ros-info "return from gripper sensor event")
               (send *ri* :move-gripper arm 0.08 :wait t))
      (progn
        (send *ri* :move-gripper arm 0.08 :wait t)
        (unix::sleep 3)))
    ;;
    (send *ri* :angle-vector-sequence (append (cdr (reverse avs)) (list av)) tms)
    (send *ri* :move-gripper arm 0.00 :wait nil)
    (send *ri* :wait-interpolation)
    ;;
    (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
    (pr2-tuckarm-pose tuckarm)
    ))

(defun tray-detection (obj)
  (let (ret)
    ;; start program
    (ros::roseus "objectdetection_tray_publisher")
    (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects :flush t))
    (while (not ret)
    (setq ret
          (check-detection :type "tray_center" ;; work for any object
                           :speak-name "tray"
                           :target-object obj
                           :timeout 30
                           :diff-position 10
                           :diff-rotation (deg2rad 10)
                           :speak t)))

    (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects :flush t))

    (if (boundp '*irtviewer*) (send ret :draw-on :flush t :size 100))
    (if (boundp '*irtviewer*) (send (send *pr2* :copy-worldcoords) :draw-on :flush t :size 2000))
    (send obj :move-to ret :world)
    (send obj :transform *pr2* :world)
    (if (boundp '*irtviewer*) (send (send obj :copy-worldcoords) :draw-on :flush t :size 1500 :color #f(1 0 0)))
    (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects :flush t))
    ret))

(provide :pr2-action)
