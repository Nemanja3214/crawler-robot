;; Auto-generated. Do not edit!


(when (boundp 'hexapod_training::QMatrix)
  (if (not (find-package "HEXAPOD_TRAINING"))
    (make-package "HEXAPOD_TRAINING"))
  (shadow 'QMatrix (find-package "HEXAPOD_TRAINING")))
(unless (find-package "HEXAPOD_TRAINING::QMATRIX")
  (make-package "HEXAPOD_TRAINING::QMATRIX"))

(in-package "ROS")
;;//! \htmlinclude QMatrix.msg.html


(defclass hexapod_training::QMatrix
  :super ros::object
  :slots (_elements ))

(defmethod hexapod_training::QMatrix
  (:init
   (&key
    ((:elements __elements) ())
    )
   (send-super :init)
   (setq _elements __elements)
   self)
  (:elements
   (&rest __elements)
   (if (keywordp (car __elements))
       (send* _elements __elements)
     (progn
       (if __elements (setq _elements (car __elements)))
       _elements)))
  (:serialization-length
   ()
   (+
    ;; hexapod_training/QMatrixElement[] _elements
    (apply #'+ (send-all _elements :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; hexapod_training/QMatrixElement[] _elements
     (write-long (length _elements) s)
     (dolist (elem _elements)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; hexapod_training/QMatrixElement[] _elements
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _elements (let (r) (dotimes (i n) (push (instance hexapod_training::QMatrixElement :init) r)) r))
     (dolist (elem- _elements)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get hexapod_training::QMatrix :md5sum-) "fdb7bde857788bc279933d9d650b6350")
(setf (get hexapod_training::QMatrix :datatype-) "hexapod_training/QMatrix")
(setf (get hexapod_training::QMatrix :definition-)
      "#QMatrix.msg
hexapod_training/QMatrixElement[] elements

================================================================================
MSG: hexapod_training/QMatrixElement
# QMatrixElement.msg
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



(provide :hexapod_training/QMatrix "fdb7bde857788bc279933d9d650b6350")


