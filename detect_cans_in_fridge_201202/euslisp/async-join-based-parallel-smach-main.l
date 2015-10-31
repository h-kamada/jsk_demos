

(ros::load-ros-manifest "roseus_smach")
(ros::roseus "async_join_state_machine_based_fridge_demo")
(require :state-machine-actionlib "package://roseus_smach/src/state-machine-actionlib.l")

(defmethod async-join-state
 (:execute
  (userdata &key (step nil))
  (let (start-time ret clients part-async-client-lst)
    (send self :remap userdata :invert nil)
    (setq start-time (ros::time-now))
    (setq part-async-client-lst (cdr (assoc :part-sync userdata)))
    (cond
      ((null part-async-client-lst) ; normal mode
       (setq clients (cdr (assoc :async userdata)))
       (when (atom clients)
         (setq clients (list clients)))
       (warning-message 2 "waiting action client: ~A~%" clients)
       (send-all clients :spin-once)
       (if (member nil (send-all clients :wait-for-result :timeout timeout))
           (setq ret nil) (setq ret t))
       (warn "ret: ~A~%" ret)
       (set-alist :results (send-all clients :get-result) userdata)
       (warn "result: ~A~%" (send-all clients :get-result))
       )
      (t ; partly join
       (warning-message 2 "<partly join>  waiting action client: ~A~%" partly-async-client-lst)
       (send-all partly-async-client-lst :spin-once)
       (if (member nil (send-all partly-async-client-lst :wait-for-result :timeout timeout))
           (setq ret nil) (setq ret t))
       (warn "ret: ~A~%" ret)
       (set-alist :results (send-all partly-async-client-lst :get-result) userdata)
       (warn "result: ~A~%" (send-all partly-async-client-lst :get-result))
         ))
    (send self :remap userdata :invert t)
    (return-from :execute ret))))


(defun actionlib-client-state
  (name client my-controller-lst
   &key (timeout 10) (retry nil) (key #'identity) (return-success :succeeded) (return-fail :failed)
        (async nil) (return-async t))
  (send client :wait-for-server)
  (instance state :init name
   ;; main loop
   `(lambda (userdata)
      (let ((start (ros::time-now)) async-clients)
  (send ,client :send-goal
        (funcall (quote ,key) (cdr (assoc :goal userdata))))
  (if ,async
    (progn
      (setq async-clients (cdr (assoc :async userdata)))
      (ros::ros-info "<exec> userdata: ~A" userdata)
      (set-alist :async (flatten (list async-clients ,client)) userdata)
      (ros::ros-info "<exec> userdata: ~A" userdata)

      ;; add client to controller-lst in userdata
      (if (not (null ,my-controller-lst))
          (progn
            (dolist (controller ,my-controller-lst)
              (let (client-lst)
                (setq client-lst (cdr (assoc controller userdata)))
                (set-alist controller (flatten (list client-lst ,client)) userdata)
              ))))

     (ros::sleep)
     (send ,client :spin-once)
     ,return-async)
  (while (ros::ok)
    (ros::sleep)
    (send ,client :spin-once)
    ;;
    (cond
     ((eq (send ,client :get-state) actionlib_msgs::GoalStatus::*SUCCEEDED*)
       (return ,return-success))
      ((eq (send ,client :get-state) actionlib_msgs::GoalStatus::*ABORTED*)
       (if ,retry
     (send ,client :send-goal
           (funcall (quote ,key) (cdr (assoc :goal userdata))))
         (return ,return-fail)))
      ((member (send ,client :get-state)
         (list actionlib_msgs::GoalStatus::*PREEMPTED*
         actionlib_msgs::GoalStatus::*RECALLED*
         actionlib_msgs::GoalStatus::*REJECTED*))
       (return ,return-fail))
      ((member (send ,client :get-state)
         (list actionlib_msgs::GoalStatus::*ACTIVE*
          actionlib_msgs::GoalStatus::*PENDING*))
       ;; user cancel
       (if (and (vectorp (cdr (assoc :cancel userdata)))
          (elt (cdr (assoc :cancel userdata)) 0))
     (send ,client :cancel-goal))
       ;; time out
       (if (and (numberp ,timeout)
          (< ,timeout (send (ros::time- (ros::time-now) start) :to-sec)))
     (send ,client :cancel-goal))))
    )))))
  )

(defun start (&optional userdata)
  (ros::ros-info "    Info: start function is being executed.")
  )

(defun convert (val)
  (let ((goal (instance roseus_smach::Sub5ActionGoal :init)))
;    (ros::ros-info "    convert to actiongoal")
    (send goal :goal :value val)
    goal))

(defun update-client-lst (userdata)
  (let ((async-client-lst (cdr (assoc :async userdata)))
        (part-async-client-lst (cdr (assoc :part-sync userdata)))
        (base-client-lst (cdr (assoc :base userdata)))
        (head-client-lst (cdr (assoc :head userdata)))
        (rarm-client-lst (cdr (assoc :rarm userdata)))
        (larm-client-lst (cdr (assoc :larm userdata)))
        (rgripper-client-lst (cdr (assoc :rgripper userdata)))
        (lgripper-client-lst (cdr (assoc :lgripper userdata)))
        (torso-client-lst (cdr (assoc :torso userdata)))
        )
    ;; checkの一番最初にupdateが呼ばれdeleteするので、join直後にdeleteしなくてもよい
    (warn "<update> userdata: ~A~%" userdata)
    (dolist (client async-client-lst)
      (send client :spin-once)
      (case (send client :get-goal-state)
        ((ros::*simple-goal-state-done*)
         (ros::ros-info "client(~A)'s goal is achived. Delete the client." client)
         ;; delete the client from all lists
         (setq async-client-lst (remove client async-client-lst)
               part-async-client-lst (remove client part-async-client-lst)
               base-client-lst (remove client base-client-lst)
               head-client-lst (remove client head-client-lst)
               rarm-client-lst (remove client rarm-client-lst)
               larm-client-lst (remove client larm-client-lst)
               rgripper-client-lst (remove client rgripper-client-lst)
               lgripper-client-lst (remove client lgripper-client-lst)
               torso-client-lst (remove client torso-client-lst)
               ))
        (t
         ;; nothing
         )
        ))
    ;; set those to userdata
    (set-alist :async async-client-lst userdata)
    (set-alist :part-async part-async-client-lst userdata)
    (set-alist :base base-client-lst userdata)
    (set-alist :head head-client-lst userdata)
    (set-alist :rarm rarm-client-lst userdata)
    (set-alist :larm larm-client-lst userdata)
    (set-alist :rgripper rgripper-client-lst userdata)
    (set-alist :lgripper lgripper-client-lst userdata)
    (set-alist :torso torso-client-lst userdata)
    (warn "<update> userdata: ~A~%" userdata)
    ))



(defmacro make-check-function (user-defined-function my-controller-lst)
  `(let ()
     (cond
       ((not (functionp ',user-defined-function))
        #'(lambda (userdata)
            ;; update
            (update-client-lst userdata)

            (ros::ros-info "    user function not defined. Check structure hazard.")
            (ros::ros-info "    userdata:~A" userdata)
            (cond
              ((equal (car (cdr (assoc :async userdata))) nil)
               t)
              (t ;; There are some clients waiting result.

               ;; currently transition to join state if controller already exists.
               (let ((part-async-client-lst (cdr (assoc :part-async userdata)))
                     )
                 (dolist (controller ,my-controller-lst)
                   (let ((client-lst (cdr (assoc controller userdata)))
                         )
                     (cond
                      ((null (car client-lst))
                       ;;no clients
                       nil)
                      (t
                       (setq part-async-client-lst (append part-async-client-lst client-lst))
                       ))))
                 (setq part-async-client-lst (remove-duplicates part-async-client-lst))
                 (set-alist :part-async part-async-client-lst userdata)
                 )
               nil
               ))))
       (t
        #'(lambda (userdata)
            ;; update
            (update-client-lst userdata)

            (ros::ros-info "    user function found. Exec user function.")
            (,user-defined-function userdata)
            t
            )))
     ))

;; (defun a-check (userdata)
;;   (ros::ros-info "a-check")
;;   t
;;   )

(defun make-hazard-remap-lst (my-controller-lst)
  (let ((remap-lst (list (:goal . random-int-value)))
        )
    (dolist (controller my-controller-lst)
      (case controller
        ((:base)
         (setq remap-lst (append remap-lst '((:base . base-clients)))))
        ((:head)
         (setq remap-lst (append remap-lst '((:head . head-clients)))))
        ((:rarm)
         (setq remap-lst (append remap-lst '((:rarm . rarm-clients)))))
        ((:larm)
         (setq remap-lst (append remap-lst '((:larm . larm-clients)))))
        ((:rgripper)
         (setq remap-lst (append remap-lst '((:rgripper . rgripper-clients)))))
        ((:lgripper)
         (setq remap-lst (append remap-lst '((:lgripper . lgripper-clients)))))
        ((:torso)
         (setq remap-lst (append remap-lst '((:torso . torso-clients)))))
        ))
    remap-lst
    ))

(defmacro make-hazard-check-state-machine (sm-name actionlib-name &optional (check-func nil) (my-controller-lst nil))
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
        (send ,sm-name :add-node (instance state :init st-check (make-check-function ,check-func ,my-controller-lst)))
        ;; checkにおける remapはuserdata情報をcheckするために必要
        (send (send ,sm-name :node st-check) :remap-list '((:async . async-clients)
                                                           (:part-async . part-async-clients)
                                                           ))

        (send ,sm-name :add-node (instance async-join-state :init st-join :timeout 30
                                           :remap-list '((:async . async-clients)
                                                         (:part-async . part-async-clients)
                                                         )))
        (send ,sm-name :add-node (actionlib-client-state st client ,my-controller-lst :async t :timeout 7 :retry t :key 'convert))
        (cond
         ((null ,my-controller-lst)
          (send (send ,sm-name :node st) :remap-list '((:goal . random-int-value)
                                                       (:part-async . part-async-clients)
                                                       (:async . async-clients)))
          )
         (t
          (send (send ,sm-name :node st) :remap-list (make-hazard-remap-lst ,my-controller-lst))
          ))
        (send ,sm-name :goal-state (list :success :failure))
        (send ,sm-name :add-transition st-check st t)
        (send ,sm-name :add-transition st-check st-join nil)
        (send ,sm-name :add-transition st-join st t)
        (send ,sm-name :add-transition st :success t)
        (send ,sm-name :add-transition st :failure nil)
        (send ,sm-name :start-state st-check)
        (send ,sm-name :arg-keys :results 'random-int-value 'async-clients 'part-async-clients)
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
        num-value
        )
    (send sm :add-node (instance state :init :start 'start))
    (send sm :add-node
          (instance state :init :goto
                    (make-hazard-check-state-machine goto-front-of-fridge "goto_front_of_fridge"))
          )
    ;; (send (send sm :node :goto) :remap-list
    ;;       '((:goal . random-int-value)
    ;;         (:async . async-clients)
    ;;         (:part-async . part-async-clients)
    ;;         (:base . base-clients)
    ;;         (:head . head-clients)
    ;;         (:rarm . rarm-clients)
    ;;         (:larm . larm-clients)
    ;;         (:rgripper . rgripper-clients)
    ;;         (:lgripper . lgripper-clients)
    ;;         (:torso . torso-clients)))
    (send sm :add-node
          (instance state :init :base
                    (make-hazard-check-state-machine base-go-to-fridge "base_go_to_fridge"))
          )
    ;; (send (send sm :node :base) :remap-list
    ;;       '((:goal . random-int-value)
    ;;         (:async . async-clients)
    ;;         (:part-async . part-async-clients)
    ;;         (:base . base-clients)
    ;;         (:head . head-clients)
    ;;         (:rarm . rarm-clients)
    ;;         (:larm . larm-clients)
    ;;         (:rgripper . rgripper-clients)
    ;;         (:lgripper . lgripper-clients)
    ;;         (:torso . torso-clients)))

    (send sm :arg-keys :results 'random-int-value 'async-clients 'part-async-clients
          'base-clients 'head-clients 'rarm-clients 'larm-clients 'rgripper-clients
          'lgripper-clients 'torso-clients
          )
    (send sm :start-state :start)
    (send sm :goal-state (list :success))
    (send sm :add-transition :start :goto t)
    (send sm :add-transition :goto :base :success)
    (Send sm :add-transition :base :success :success)

    (setq *sm* sm)
    sm))

(defun demo ()
  (make-sm)
  (smach-exec *sm*))

(warn "~%")
(warn "~%")
(warn "Caution######################################################~%")
(warn "async-join-state class differs from original one.~%")
(warn "actionlib-client-state function differs from original one.~%")
(warn "ros::simple-action-client differs from original one. <--- changed raw data ~%")
(warn "#############################################################~%")
(warn "~%")
(warn "~%")









