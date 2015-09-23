

(load "package://pr2eus/pr2-interface.l")
(pr2-init)
(load "pr2-action.l")
(load "package://jsk_perception/euslisp/detection_interface.l")
(load "package://jsk_perception/euslisp/detection_interface_servicecall.l")

(setq *type* "georgia")

(open-fridge-door :use-arm :larm)
(grasp-can :use-arm :larm)
(close-fridge :use-arm :larm)