;; Auto-generated. Do not edit!


(when (boundp 'hexapod_training::QMatrixElement)
  (if (not (find-package "HEXAPOD_TRAINING"))
    (make-package "HEXAPOD_TRAINING"))
  (shadow 'QMatrixElement (find-package "HEXAPOD_TRAINING")))
(unless (find-package "HEXAPOD_TRAINING::QMATRIXELEMENT")
  (make-package "HEXAPOD_TRAINING::QMATRIXELEMENT"))

(in-package "ROS")
;;//! \htmlinclude QMatrixElement.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass hexapod_training::QMatrixElement
  :super ros::object
  :slots (_pair _reward ))

(defmethod hexapod_training::QMatrixElement
  (:init
   (&key
    ((:pair __pair) (instance hexapod_training::StateActionPair :init))
    ((:reward __reward) (instance std_msgs::Float64 :init))
    )
   (send-super :init)
   (setq _pair __pair)
   (setq _reward __reward)
   self)
  (:pair
   (&rest __pair)
   (if (keywordp (car __pair))
       (send* _pair __pair)
     (progn
       (if __pair (setq _pair (car __pair)))
       _pair)))
  (:reward
   (&rest __reward)
   (if (keywordp (car __reward))
       (send* _reward __reward)
     (progn
       (if __reward (setq _reward (car __reward)))
       _reward)))
  (:serialization-length
   ()
   (+
    ;; hexapod_training/StateActionPair _pair
    (send _pair :serialization-length)
    ;; std_msgs/Float64 _reward
    (send _reward :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; hexapod_training/StateActionPair _pair
       (send _pair :serialize s)
     ;; std_msgs/Float64 _reward
       (send _reward :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; hexapod_training/StateActionPair _pair
     (send _pair :deserialize buf ptr-) (incf ptr- (send _pair :serialization-length))
   ;; std_msgs/Float64 _reward
     (send _reward :deserialize buf ptr-) (incf ptr- (send _reward :serialization-length))
   ;;
   self)
  )

(setf (get hexapod_training::QMatrixElement :md5sum-) "39b6677d577d14397b6281d04d889aab")
(setf (get hexapod_training::QMatrixElement :datatype-) "hexapod_training/QMatrixElement")
(setf (get hexapod_training::QMatrixElement :definition-)
      "# QMatrixElement.msg
hexapod_training/StateActionPair pair
std_msgs/Float64 reward

================================================================================
MSG: hexapod_training/StateActionPair
# StateActionPair.msg
string state
int32 action
================================================================================
MSG: std_msgs/Float64
float64 data
")



(provide :hexapod_training/QMatrixElement "39b6677d577d14397b6281d04d889aab")


