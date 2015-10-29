

(ros::load-ros-manifest "roseus_smach")
(ros::roseus "async_join_state_machine_based_fridge_demo")
(require :state-machine-actionlib "package://roseus_smach/src/state-machine-actionlib.l")

(defun start (&optional userdata)
  (ros::ros-info "start")
  )

(defun convert (val)
  (let ((goal (instance roseus_smach::Sub5ActionGoal :init)))
    (ros::ros-info "convert to actiongoal")
    (send goal :goal :value val)
    goal))

(defvar random-int-value nil)
(defvar async-clients nil)

(defmacro make-check-function (user-defined-function)
  `(let ()
     (cond
       ((not (functionp ',user-defined-function))
        #'(lambda (userdata)
            (ros::ros-info "user function not defined. Just check structure hazard.")
            (cond
              ((equal (car (cdr (assoc :async userdata))) nil)
               t)
              (t
               nil
               ))))
       (t
        #'(lambda (userdata)
            (ros::ros-info "user function found. Exec user function.")
            (,user-defined-function userdata)
            t
            )))
     ))

(defun a-check (userdata)
  (ros::ros-info "a-check")
  t
  )

(defmacro make-hazard-check-state-machine (sm-name actionlib-name &optional (check-func nil))
  `(let (st st-check st-join
         )
     (cond
       ((boundp ',sm-name)
        (ros::ros-warn "state-machine(~A) is already created." ,sm-name)
        ,sm-name
        )
       (t
        (setq ,sm-name (instance state-machine :init))
        (setq st (intern (string ',sm-name) *keyword-package*))
        (setq st-check (intern (string-upcase (format nil "~A-check" ',sm-name)) *keyword-package*))
        (setq st-join (intern (string-upcase (format nil "~A-join" ',sm-name)) *keyword-package*))
        (setq client (instance ros::simple-action-client :init
                               ,actionlib-name roseus_smach::Sub5Action))
        (setq ,sm-name (instance state-machine :init))
        (send ,sm-name :add-node (instance state :init st-check (make-check-function ,check-func)))
        (send ,sm-name :add-node (instance async-join-state :init st-join :timeout 30
                                           :remap-list '((:async . async-clients))))
        (send ,sm-name :add-node (actionlib-client-state st client :async t :timeout 7 :retry t :key 'convert))
        (send (send ,sm-name :node st) :remap-list '((:goal . random-int-value)
                                                     (:async . async-clients)))
        (send ,sm-name :goal-state (list :success :failure))
        (send ,sm-name :add-transition st-check st t)
        (send ,sm-name :add-transition st-check st-join nil)
        (send ,sm-name :add-transition st-join st t)
        (send ,sm-name :add-transition st :success t)
        (send ,sm-name :add-transition st :failure nil)
        (send ,sm-name :start-state st-check)
        (send ,sm-name :arg-keys :results 'random-int-value 'async-clients)
        ,sm-name
     ))))

(defun exm (&optional arg)
  (cond
    (arg
       (smach-exec (make-hazard-check-state-machine a "goto_front_of_fridge")))
    (t
     (smach-exec (make-hazard-check-state-machine a "goto_front_of_fridge" a-check)))))

;; defun version

;; (defun mk (state-name client check-func)
;;   (let ((sm (instance state-machine :init))
;;         st st-check st-join
;;         )
;;   (setq st (intern (string state-name) *keyword-package*))
;;   (setq st-check (intern (string-upcase (format nil "~A-check" state-name)) *keyword-package*))
;;   (setq st-join (intern (string-upcase (format nil "~A-join" state-name)) *keyword-package*))
;;   (send sm :add-node (instance state :init st-check check-func))
;;   (send sm :add-node (instance async-join-state :init st-join :timeout 30
;;                                :remap-list '((:async . async-clients))))
;;   (send sm :add-node (actionlib-client-state st client :async t :timeout 7 :retry t
;;                                              :key 'convert))
;;   (send (send sm :node st) :remap-list '((:goal . random-int-value)
;;                                          (:async . async-clients)))
;;   (send sm :goal-state (list :success :failure))
;;   (send sm :add-transition st-check st t)
;;   (send sm :add-transition st-check st-join nil)
;;   (send sm :add-transition st-join st t)
;;   (send sm :add-transition st :success t)
;;   (send sm :add-transition st :failure nil)
;;   (send sm :start-state st-check)
;;   (send sm :arg-keys :results 'random-int-value 'async-clients)
;;   sm
;;   ))

;; (defun ex ()
;;   (smach-exec (mk 'a a-client 'a-check)))

(defun make-sm ()
  (let ((sm (instance state-machine :init))
        ;; (goto-front-of-fridge-sm (instance state-machine :init))
        ;; (base-go-to-front-of-fridge-sm (instance state-machine :init))
        ;; (open-fridge-door-initial-pose-sm (instance state-machine :init))
        ;; (move-to-and-open-fridge-door-sm (instance state-machine :init))
        ;; (move-forward-larm-sm (instance state-machine :init))
        ;; (detatch-fridge-handle-sm (instance state-machine :init))
        ;; (swipe-fridge-door-sm (instance state-machine :init))
        ;; (move-to-can-spot-sm (instance state-machine :init))
        ;; (grasp-can-init-sm (instance state-machine :init))
        ;; (grasp-can-motion-sm (instance state-machine :init))
        ;; (go-back-from-fridge-sm (instance state-machine :init))
        ;; (close-fridge-sm (instance state-machine :init))

        ;; (goto-front-of-fridge-client (instance ros::simple-action-client :init
        ;;                                        "goto_front_of_fridge" roseus_smach::Sub5Action))
        ;; (base-go-to-fridge-client (instance ros::simple-action-client :init
        ;;                                     "base_go_to_fridge" roseus_smach::Sub5Action))
        ;; (open-fridge-door-initial-pose-client
        ;;  (instance ros::simple-action-client :init
        ;;            "open_fridge_door_initial_pose" roseus_smach::Sub5Action))
        ;; (move-to-and-open-fridge-door-client
        ;;  (instance ros::simple-action-client :init
        ;;            "move_to_and_open_fridge_door" roseus_smach::Sub5Action))
        ;; (move-forward-larm-client (instance ros::simple-action-client :init
        ;;                                     "move_forward_larm" roseus_smach::Sub5Action))
        ;; (detach-fridge-handle-client (instance ros::simple-action-client :init
        ;;                                        "detach_fridge_handle" roseus_smach::Sub5Action))
        ;; (swipe-fridge-door-client (instance ros::simple-action-client :init
        ;;                                        "swipe_fridge_door" roseus_smach::Sub5Action))
        ;; (move-to-can-spot-client (instance ros::simple-action-client :init
        ;;                                    "move_to_can_spot" roseus_smach::Sub5Action))
        ;; (grasp-can-init-client (instance ros::simple-action-client :init
        ;;                                  "grasp_can_init" roseus_smach::Sub5Action))
        ;; (grasp-can-motion-client (instance ros::simple-action-client :init
        ;;                                  "grasp_can_motion" roseus_smach::Sub5Action))n
        ;; (go-back-from-fridge-client (instance ros::simple-action-client :init
        ;;                                       "go_back_from_fridge" roseus_smach::Sub5Action))
        ;; (close-fridge-client (instance ros::simple-action-client :init
        ;;                                "close_fridge" roseus_smach::Sub5Action))

        num-value
        async-clients)


    (send sm :add-node (instance state :init :start 'start))

    (send sm :add-node
          (make-hazard-check-state-machine goto-front-of-fridge "goto_front_of_fridge")
          )
    ;; (send goto-front-of-fridge-sm :add-node (instance state :init :goto-front-of-fridge-check 'goto-front-of-fridge-check))
    ;; (send goto-front-of-fridge-sm :add-node (instance async-join-state :init :join :timeout 30 :remap-list '((:async . async-clients))))

    ;; (send goto-front-of-fridge-sm :add-node (actionlib-client-state
    ;;                                          :goto-front-of-fridge goto-front-of-fridge-client :async t
    ;;                                          :timeout 7 :retry t :key 'convert))
    ;; (send (send goto-front-of-fridge-sm :node :goto-front-of-fridge) :remap-list '((:goal . random-int-value)
    ;;                                                                                (:async . async-clients)))
    ;; (send sm :add-node goto-front-of-fridge-sm)

    (send sm :add-node
          (make-hazard-check-state-machine base-go-to-fridge "base_go_to_fridge")
          )
    ;; (send base-go-to-fridge-sm :add-node (instance state :init :base-go-to-fridge-check 'base-go-to-fridge-check))
    ;; (send base-go-to-fridge-sm :add-node (instance state :init :base-go-to-fridge-join 'goto-
    ;; (send base-go-to-fridge-sm :add-node
    ;;       (actionlib-client-state
    ;;        :base-go-to-fridge base-go-to-fridge-client :async t
    ;;        :timeout 7 :retry t :key 'convert))
    ;; (send (send base-go-to-fridge-sm :node :base-go-to-fridge) :remap-list
    ;;       '((:goal . random-int-value)
    ;;         (:async . async-clients)))
    ;; (send sm :add-node
    ;;       (instance async-join-state :init :join :timeout 30
    ;;                 :remap-list '((:async . async-clients))))


    ;; (send sm :add-node
    ;;       (actionlib-client-state
    ;;        :open-fridge-door-initial-pose open-fridge-door-initial-pose-client :async t
    ;;        :timeout 7 :retry t :key 'convert))
    ;; (send sm :add-node
    ;;       (actionlib-client-state
    ;;        :move-to-and-open-fridge-door move-to-and-open-fridge-door-client :async t
    ;;        :timeout 7 :retry t :key 'convert))
    ;; (send sm :add-node
    ;;       (actionlib-client-state
    ;;        :move-forward-larm move-forward-larm-client :async t
    ;;        :timeout 7 :retry t :key 'convert))
    ;; (send sm :add-node
    ;;       (actionlib-client-state
    ;;        :detach-fridge-handle detach-fridge-handle-client :async t
    ;;        :timeout 7 :retry t :key 'convert))
    ;; (send sm :add-node
    ;;       (actionlib-client-state
    ;;        :swipe-fridge-door swipe-fridge-door-client :async t
    ;;        :timeout 7 :retry t :key 'convert))
    ;; (send sm :add-node
    ;;       (actionlib-client-state
    ;;        :move-to-can-spot move-to-can-spot-client :async t
    ;;        :timeout 7 :retry t :key 'convert))
    ;; (send sm :add-node
    ;;       (actionlib-client-state
    ;;        :grasp-can-init grasp-can-init-client :async t
    ;;        :timeout 7 :retry t :key 'convert))
    ;; (send sm :add-node
    ;;       (actionlib-client-state
    ;;        :grasp-can-motion grasp-can-motion-client :async t
    ;;        :timeout 7 :retry t :key 'convert))
    ;; (send sm :add-node
    ;;       (actionlib-client-state
    ;;        :go-back-from-fridge go-back-from-fridge-client :async t
    ;;        :timeout 7 :retry t :key 'convert))
    ;; (send sm :add-node
    ;;       (actionlib-client-state
    ;;        :close-fridge close-fridge-client :async t
    ;;        :timeout 7 :retry t :key 'convert))

    (ros::ros-info "hoge")
    (send sm :arg-keys :results 'random-int-value 'async-clients)
    (send sm :start-state :start)
    (send sm :goal-state (list :success))
    (send sm :add-transition :start goto-front-of-fridge t)
    (ros::ros-info "fuga")
    (send sm :add-transition goto-front-of-fridge base-go-to-fridge t)
    (ros::ros-info "1a")
    (send sm :add-transition base-go-to-fridge :success t)
    ;; (send sm :add-transition :start :base-go-to-fridge t)
    ;; (send sm :add-transition :goto-front-of-fridge :join t)
    ;; (send sm :add-transition :base-go-to-fridge :join t)
    ;; (send sm :add-transition :join :open-fridge-door-initial-pose t)
    ;; (send sm :add-transition :open-fridge-door-initial-pose :move-to-and-open-fridge-door t)
    ;; (send sm :add-transition :move-to-and-open-fridge-door :move-forward-larm t)
    ;; (send sm :add-transition :move-forward-larm :detach-fridge-handle t)
    ;; (send sm :add-transition :detach-fridge-handle :swipe-fridge-door t)
    ;; (send sm :add-transition :swipe-fridge-door :move-to-can-spot t)
    ;; (send sm :add-transition :move-to-can-spot :grasp-can-init t)
    ;; (send sm :add-transition :grasp-can-init :grasp-can-motion t)
    ;; (send sm :add-transition :grasp-can-motion :go-back-from-fridge t)
    ;; (send sm :add-transition :go-back-from-fridge :close-fridge t)
    ;; (send sm :add-transition :close-fridge :success t)

    (setq *sm* sm)
    sm))

(defun demo ()
  (make-sm)
  (smach-exec *sm*))
