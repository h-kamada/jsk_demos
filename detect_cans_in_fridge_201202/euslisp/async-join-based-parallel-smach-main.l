

(ros::load-ros-manifest "roseus_smach")
(ros::roseus "async_join_state_machine_based_fridge_demo")
(require :state-machine-actionlib "package://roseus_smach/src/state-machine-actionlib.l")

(defmethod async-join-state
 (:execute
  (userdata &key (step nil))
  (let (start-time ret clients part-async-client-lst)
;    (ros::ros-info "userdata:~A" userdata)
    (send self :remap userdata :invert nil)
;    (ros::ros-info "hogehoge userdata:~A" userdata)
    (setq start-time (ros::time-now))
    (setq part-async-client-lst (cdr (assoc :part-async userdata)))
;    (ros::ros-info "part-async: ~A" part-async-client-lst)
    (cond
      ((null part-async-client-lst) ; normal mode
       (setq clients (cdr (assoc :async userdata)))
       (when (atom clients)
         (setq clients (list clients)))
       (warning-message 2 "<normal async join> waiting action client: ~A~%" clients)
       (send-all clients :spin-once)
       (if (member nil (send-all clients :wait-for-result :timeout timeout))
           (setq ret nil) (setq ret t))
       (warn "ret: ~A~%" ret)
       (set-alist :results (send-all clients :get-result) userdata)
       (warn "result: ~A~%" (send-all clients :get-result))
       )
      (t ; partly join
       (warning-message 2 "<part async join> waiting action client:~A" part-async-client-lst)
       (send-all part-async-client-lst :spin-once)
       (if (member nil (send-all part-async-client-lst :wait-for-result :timeout timeout))
           (setq ret nil) (setq ret t))
       (warn "ret: ~A~%" ret)
       (set-alist :results (send-all part-async-client-lst :get-result) userdata)
       (warn "result: ~A~%" (send-all part-async-client-lst :get-result))
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
;      (ros::ros-info "<exec> userdata: ~A" userdata)
      (set-alist :async (flatten (list async-clients ,client)) userdata)
;      (ros::ros-info "<exec> userdata: ~A" userdata)

      ;; add client to controller-lst in userdata
;      (ros::ros-info "<exec> my-controller-lst: ~A" (car ',my-controller-lst))
      (if (not (null ',my-controller-lst))
          (progn
            (dolist (controller ',my-controller-lst)
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
  :success
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
 ;   (warn "<update> userdata: ~A~%" userdata)
    ))

;; (defun set-part-async-client-lst (userdata my-controller-lst)
;;   (let ((part-async-client-lst (cdr (assoc :part-async userdata))) ; nilのはずではあるが一応
;;         hazard-assoc-lst
;;         )
;;     (setq hazard-assoc-lst
;;           '((:base :rgripper :lgripper)
;;             (:head)
;;             (:rarm :rgripper :base)
;;             (:larm :lgripper :base)
;;             (:rgripper :rarm :base :torso)
;;             (:lgripper :larm :base :torso)
;;             (:torso :rgripper :lgripper)
;;             ))
;;     (ros::ros-info "mycon:~A" my-controller-lst)
;;     (setq *aa* my-controller-lst)
;;     (dolist (controller my-controller-lst)
;;       (ros::ros-info "controller: ~A" controller)
;;       (dolist (ctr (assoc controller hazard-assoc-lst))
;;         (ros::ros-info "ctr: ~A" ctr)
;;         (let ((client-lst (cdr (assoc ctr userdata))))
;;           (ros::ros-info "clie-lst: ~A" client-lst)
;;           (cond
;;            ((null (car client-lst))
;;             ;;no clients
;;             )
;;            (t
;;             (setq part-async-client-lst (append part-async-client-lst client-lst))
;;             )))))
;;     (setq part-async-client-lst (remove-duplicates part-async-client-lst))
;;     (ros::ros-info "    <check> userdata:~A" userdata)
;;     (set-alist :part-async part-async-client-lst userdata)
;;     (ros::ros-info "    <check> userdata:~A" userdata)
;;     ))

(defun set-part-async-client-lst (userdata my-controller-lst)
  (let ((part-async-client-lst (cdr (assoc :part-async userdata))) ; nilのはずではあるが一応
        hazard-assoc-lst
        )
    (setq hazard-assoc-lst
          '((:base :rgripper :lgripper)
            (:head)
            (:rarm :rgripper :base)
            (:larm :lgripper :base)
            (:rgripper :rarm :base :torso)
            (:lgripper :larm :base :torso)
            (:torso :rgripper :lgripper)
            ))
    (ros::ros-info "set mycon:~A" my-controller-lst)
    (dolist (controller my-controller-lst)
      (ros::ros-info "controller: ~A" controller)
      (dolist (ctr (assoc controller hazard-assoc-lst))
        (ros::ros-info "ctr: ~A" ctr)
        (let ((client-lst (cdr (assoc ctr userdata))))
          (ros::ros-info "clie-lst: ~A" client-lst)
          (cond
           ((null (car client-lst))
            ;;no clients
            )
           (t
            (setq part-async-client-lst (append part-async-client-lst client-lst))
            )))))
    (setq part-async-client-lst (remove-duplicates part-async-client-lst))
    (ros::ros-info "    <check> userdata:~A" userdata)
    (set-alist :part-async part-async-client-lst userdata)
    (ros::ros-info "    <check> userdata:~A" userdata)
    ))

;; (defmacro make-check-function (user-defined-function my-controller-lst)
;;   (ros::ros-info "check: mycon: ~A" my-controller-lst)
;;   `(let ()
;;      (cond
;;        ((not (functionp ',user-defined-function))
;;         (ros::ros-info "mycon  bbb: ~A" ,my-controller-lst)
;;         #'(lambda (userdata)
;;             ;; update
;;             (update-client-lst userdata)
;;             (ros::ros-info "    user function not defined. Check structure hazard.")
;;             (cond
;;               ((equal (car (cdr (assoc :async userdata))) nil)
;;                t)
;;               (t ;; There are some clients waiting result.
;;                (set-part-async-client-lst userdata ,my-controller-lst)
;;                (ros::ros-info "mycon: ~A" ,my-controller-lst)
;; ;               (set-part-async-client-lst userdata ,my-controller-lst)
;;                (let ((part-async-client-lst (cdr (assoc :part-async userdata))))
;;                  (ros::ros-info "part:~A" part-async-client-lst)
;;                  (cond
;;                   ((null part-async-client-lst)
;;                    t)
;;                   (t
;;                    nil))
;;                  )
;;                ))))
;;        (t
;;         #'(lambda (userdata)
;;             ;; update
;;             (update-client-lst userdata)
;;             (ros::ros-info "    user function found. Exec user function.")
;;             (,user-defined-function userdata)
;;             t
;;             )))
;;      ))

(defmacro make-check-function (user-defined-function my-controller-lst)
  `(let ()
     (ros::ros-info "mycon: ~A" ',my-controller-lst)
     (ros::ros-info "car: ~A" (car ',my-controller-lst))
     (ros::ros-info "cdr: ~A" (cdr ',my-controller-lst))
     (ros::ros-info "user func: ~A" ',user-defined-function)
     (cond
      ((not (functionp ',user-defined-function))
       '(lambda (userdata)
          ;; update
          (update-client-lst userdata)
          (ros::ros-info "    user function not defined. Check structure hazard.")
          (cond
           ((equal (car (cdr (assoc :async userdata))) nil)
            t)
           (t ;; There are some clients waiting result.
            (set-part-async-client-lst userdata ',my-controller-lst)
            (let ((part-async-client-lst (cdr (assoc :part-async userdata)))
                  )
              (cond
               ((null part-async-client-lst)
                t)
               (t
                nil))
              )))))
      (t
       '(lambda (userdata)
          ;; update
          (update-client-lst userdata)
          (ros::ros-info "    user function found. Exec user function.")
          (,user-defined-function userdata)
          t
          )))
     ))

(defun make-hazard-remap-lst (&optional my-controller-lst)
  (let ((remap-lst '((:async . async-clients)
                     (:part-async . part-async-clients))))
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

;; (defmacro make-async-join-based-structure-hazard-check-state-machine
;;   (sm-name &optional (my-controller-lst nil) (check-func nil))
;;   "make hazard-check-state-machine"
;;   `(let (st-check st-join st-exec ac-name ;; new-name
;;          )
;;      (cond
;;       ((boundp ',sm-name)
;;         (ros::ros-warn "state-machine(~A) is already created." ,sm-name)
;;         ,sm-name
;;         )
;;        (t
;;         (ros::ros-info "mycon:~A" ,my-controller-lst)
;;         (setq ,sm-name (instance state-machine :init))
;;         (setq st-check (intern (string-upcase (format nil "~A-check" ',sm-name)) *keyword-package*))
;;         (setq st-join (intern (string-upcase (format nil "~A-join" ',sm-name)) *keyword-package*))
;;         (setq st-exec (intern (string-upcase (format nil "~A-exec" ',sm-name)) *keyword-package*))
;;         (setq ac-name  (string-downcase (substitute  #\_ #\- (string ',sm-name))))
;;         (setq client (instance ros::simple-action-client :init
;;                                ac-name roseus_smach::Sub5Action))
;;         (setq ,sm-name (instance state-machine :init))
;;         (send ,sm-name :add-node (instance state :init st-check (make-check-function ,check-func ,my-controller-lst)))

;;         (send (send ,sm-name :node st-check) :remap-list (make-hazard-remap-lst ,my-controller-lst))
;;         (send ,sm-name :add-node (instance async-join-state :init st-join :timeout 30
;;                                            :remap-list (make-hazard-remap-lst)))

;;         (send ,sm-name :add-node (actionlib-client-state st-exec client ,my-controller-lst :async t :timeout 7 :retry t :key 'convert))
;;         (send (send ,sm-name :node st-exec) :remap-list (make-hazard-remap-lst ,my-controller-lst))
;;         (send ,sm-name :goal-state (list :success :failure))
;;         (send ,sm-name :add-transition st-check st-exec t)
;;         (send ,sm-name :add-transition st-check st-join nil)
;;         (send ,sm-name :add-transition st-join st-exec t)
;;         (send ,sm-name :add-transition st-exec :success t)
;;         (send ,sm-name :add-transition st-exec :failure nil)
;;         (send ,sm-name :start-state st-check)
;;         (send ,sm-name :arg-keys
;;           'async-clients 'part-async-clients 'base-clients
;;           'head-clients 'rarm-clients 'larm-clients
;;           'rgripper-clients 'lgripper-clients 'torso-clients)
;;         ,sm-name
;;      ))))

(defmacro make-async-join-based-structure-hazard-check-state-machine
  (sm-name &optional (my-controller-lst nil) (check-func nil))
  "make hazard-check-state-machine"
  `(let (st-check st-join st-exec ac-name)
     (ros::ros-info "sm-name: ~A" ',sm-name)
     (ros::ros-info "mycon aa: ~A" ',my-controller-lst)
     (cond
      ((boundp ',sm-name)
       (ros::ros-warn "state-machine(~A) is already created." ',sm-name)
       ,sm-name
       )
      (t
       (setq st-check (intern (string-upcase (format nil "~A-check" ',sm-name)) *keyword-package*))
       (setq st-join (intern (string-upcase (format nil "~A-join" ',sm-name)) *keyword-package*))
       (setq st-exec (intern (string-upcase (format nil "~A-exec" ',sm-name)) *keyword-package*))
       (ros::ros-info "check: ~A" st-check)
       ;; (setq ac-name  (string-downcase (substitute  #\_ #\- (string ',sm-name))))
       (setq ac-name  (string-downcase (substitute  #\_ #\- (format nil "~A" ',sm-name))))
       (ros::ros-info "ac-name: ~A" ac-name)
       (setq ,sm-name (instance state-machine :init))
       (setq client (instance ros::simple-action-client :init
                              ac-name roseus_smach::Sub5Action))
       (ros::ros-info "befor mycon: ~A" ',my-controller-lst)
                                        ;       (setq ,dummy-con ,my-controller-lst)
       (send ,sm-name :add-node
             (instance state :init st-check
                       (make-check-function ,check-func ,my-controller-lst)))
       (ros::ros-info "hoge func : ~A"
                      (make-check-function ,check-func ,my-controller-lst))
       (send (send ,sm-name :node st-check) :remap-list (make-hazard-remap-lst ',my-controller-lst))
       (send ,sm-name :add-node (instance async-join-state :init st-join :timeout 30
                                                 :remap-list (make-hazard-remap-lst)))
       (send ,sm-name :add-node (actionlib-client-state st-exec client ',my-controller-lst :async t :timeout 7 :retry t :key 'convert))
       (send (send ,sm-name :node st-exec) :remap-list (make-hazard-remap-lst ',my-controller-lst))
       (send ,sm-name :goal-state (list :success :failure))
       (send ,sm-name :add-transition st-check st-exec t)
       (send ,sm-name :add-transition st-check st-join nil)
       (send ,sm-name :add-transition st-join st-exec t)
       (send ,sm-name :add-transition st-exec :success t)
       (send ,sm-name :add-transition st-exec :failure nil)
       (send ,sm-name :start-state st-check)
       (send ,sm-name :arg-keys
             'async-clients 'part-async-clients 'base-clients
             'head-clients 'rarm-clients 'larm-clients
             'rgripper-clients 'lgripper-clients 'torso-clients)
       ,sm-name
       ))))

;; (defun make-sm ()
;;   (let ((sm (instance state-machine :init))
;;         )
;;     (send sm :add-node (instance state :init :start 'start))
;;     (send sm :add-node
;;           (instance state :init :goto
;;                     (make-async-join-based-structure-hazard-check-state-machine
;;                      goto-front-of-fridge
;;                      '(:rarm :rgripper :larm :lgripper)
;;                      )))
;;     (send sm :add-node
;;           (instance state :init :base
;;                     (make-async-join-based-structure-hazard-check-state-machine
;;                      base-go-to-fridge
;;                      '(:base :rarm)
;;                      )))
;;     (send sm :arg-keys
;;           'async-clients 'part-async-clients 'base-clients
;;           'head-clients 'rarm-clients 'larm-clients
;;           'rgripper-clients 'lgripper-clients 'torso-clients)
;;     (send sm :start-state :start)
;;     (send sm :goal-state (list :success))
;;     (send sm :add-transition :start :goto :success)
;;     (send sm :add-transition :goto :base :success)
;;     (send sm :add-transition :base :success :success)
;;     (setq *sm* sm)
;;     sm))

;; (defun hoge ()
;;    (make-hoge
;;     '((goto-front-of-fridge (:rarm :larm))
;;       (base-go-to-fridge (:base :larm))
;;       ))
;;    )

;; (defun he ()
;;   (smach-exec (make-hoge
;;                '((goto-front-of-fridge (:rarm :larm))
;;                  (base-go-to-fridge (:base :larm))
;;                  ))))

(defmacro make-all-state-machine (state-machine-information-lst)
  (let ((sm (instance state-machine :init))
        (prev-sm (gensym))
        (next-sm (gensym))
        )
    `(let (state-machine-lst)
       (send ,sm :add-node (instance state :init :start 'start))
       (dolist (sm-info ,state-machine-information-lst)
         (ros::ros-info "sm-info: ~A car: ~A cdr: ~A" sm-info (car sm-info) (cdr sm-info))
         (send ,sm :add-node
               (instance state :init (intern (string (car sm-info)) *keyword-package*)
                         (eval `(make-async-join-based-structure-hazard-check-state-machine
                                 ,(car sm-info) ,(cadr sm-info)))))
         (setq state-machine-lst (append state-machine-lst (list (intern (string (car sm-info)) *keyword-package*))))
         )
       (setq *aa* ,sm)
       (send ,sm :arg-keys
             'async-clients 'part-async-clients 'base-clients
             'head-clients 'rarm-clients 'larm-clients
             'rgripper-clients 'lgripper-clients 'torso-clients)
       (send ,sm :start-state :start)
       (send ,sm :goal-state (list :success))
       (ros::ros-info "sm-lst" state-machine-lst)
       (setq prev-sm :start)
       (dolist (st-mn state-machine-lst)
         (setq next-sm st-mn)
         (send ,sm :add-transition prev-sm next-sm :success)
         (setq prev-sm st-mn)
         )
       (setq next-sm :success)
       (send ,sm :add-transition prev-sm next-sm :success)
       ,sm
       (smach-exec ,sm)
       )))

(defun demo ()
  (make-all-state-machine
    '((goto-front-of-fridge (:rarm :torso))
      (base-go-to-fridge (:base :larm))
      )
    ))

(warn "~%")
(warn "~%")
(warn "Caution######################################################~%")
(warn "async-join-state class differs from original one.~%")
(warn "actionlib-client-state function differs from original one.~%")
(warn "ros::simple-action-client differs from original one. <--- changed raw data ~%")
(warn "#############################################################~%")
(warn "~%")
(warn "~%")

