;; Auto-generated. Do not edit!


(when (boundp 'hexapod_training::StateActionPair)
  (if (not (find-package "HEXAPOD_TRAINING"))
    (make-package "HEXAPOD_TRAINING"))
  (shadow 'StateActionPair (find-package "HEXAPOD_TRAINING")))
(unless (find-package "HEXAPOD_TRAINING::STATEACTIONPAIR")
  (make-package "HEXAPOD_TRAINING::STATEACTIONPAIR"))

(in-package "ROS")
;;//! \htmlinclude StateActionPair.msg.html


(defclass hexapod_training::StateActionPair
  :super ros::object
  :slots (_state _action ))

(defmethod hexapod_training::StateActionPair
  (:init
   (&key
    ((:state __state) "")
    ((:action __action) 0)
    )
   (send-super :init)
   (setq _state (string __state))
   (setq _action (round __action))
   self)
  (:state
   (&optional __state)
   (if __state (setq _state __state)) _state)
  (:action
   (&optional __action)
   (if __action (setq _action __action)) _action)
  (:serialization-length
   ()
   (+
    ;; string _state
    4 (length _state)
    ;; int32 _action
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _state
       (write-long (length _state) s) (princ _state s)
     ;; int32 _action
       (write-long _action s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _state
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _state (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int32 _action
     (setq _action (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get hexapod_training::StateActionPair :md5sum-) "ea8f4a97a7ca851b7f7f92f5d00b7f93")
(setf (get hexapod_training::StateActionPair :datatype-) "hexapod_training/StateActionPair")
(setf (get hexapod_training::StateActionPair :definition-)
      "# StateActionPair.msg
string state
int32 action
")



(provide :hexapod_training/StateActionPair "ea8f4a97a7ca851b7f7f92f5d00b7f93")


