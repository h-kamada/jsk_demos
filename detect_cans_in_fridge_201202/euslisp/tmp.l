

(ros::load-ros-manifest "roseus_smach")
(ros::roseus "async_join_parallel_state_machine_based_fridge_demo")
(require :state-machine-actionlib "package://roseus_smach/src/state-machine-actionlib.l")

;; actionlib-client-state funcは、hoge, fugaの返り値としてのaction goal を、action client に対して投げる。そのため、hoge, fugaの返り値はaction goal でなければならない                                       
(defun hoge (val)
  (let ((goal (instance roseus_smach::Multiply10ActionGoal :init)))
    (ros::ros-info "hoge function")
    (send goal :goal :value val)
    goal))

(defun fuga (val)
  (let ((goal (instance roseus_smach::Sub5ActionGoal :init)))
    (ros::ros-info "fuga function")
    (send goal :goal :value val)
    goal))

(defun start (&optional userdata)
  (ros::ros-info "start")
  )


(defun make-sm ()
  (let ((sm (instance state-machine :init))
        (hoge-client (instance ros::simple-action-client :init
                               "hogehoge" roseus_smach::Multiply10Action))
        (fuga-client (instance ros::simple-action-client :init
                               "fugafuga" roseus_smach::Sub5Action))
        num-value
        async-clients)

    (send sm :add-node
          (instance state :init :start 'start))
    (send sm :add-node
          (actionlib-client-state
           :hoge hoge-client :async t :timeout 20 :retry t
           :key 'hoge))
    (send (send sm :node :hoge) :remap-list '((:goal . random-int-value)
                                              (:async . async-clients)
                                              ))
    ;; :goalはactionlibclientstate function の中で、assoc :goalとしているので、ここで対応するものを引数として判断し、:key として指定されているfunction の引数として渡す形となっている                        
    (send sm :add-node
          (actionlib-client-state
           :fuga fuga-client :async t :timeout 7 :retry t
           :key 'fuga))
    (send (send sm :node :fuga) :remap-list '((:goal . random-int-value)
                                              (:async . async-clients)
                                              ))
    (send sm :add-node
          (instance async-join-state :init
                    :join :timeout 30
                    :remap-list '((:async . async-clients))))
    (send sm :arg-keys :results 'random-int-value 'async-clients)
    (send sm :start-state :start)
    (send sm :goal-state (list :success))
    (send sm :add-transition :start :hoge t)
    (send sm :add-transition :start :fuga t)
    (send sm :add-transition :hoge :join t)
    (send sm :add-transition :fuga :join t)
    (send sm :add-transition :join :success t)
    (setq *sm* sm)
    sm))

(defun demo ()
  (make-sm)
  (smach-exec *sm*))
