;; Auto-generated. Do not edit!


(when (boundp 'hexapod_training::ResultMsg)
  (if (not (find-package "HEXAPOD_TRAINING"))
    (make-package "HEXAPOD_TRAINING"))
  (shadow 'ResultMsg (find-package "HEXAPOD_TRAINING")))
(unless (find-package "HEXAPOD_TRAINING::RESULTMSG")
  (make-package "HEXAPOD_TRAINING::RESULTMSG"))

(in-package "ROS")
;;//! \htmlinclude ResultMsg.msg.html


(defclass hexapod_training::ResultMsg
  :super ros::object
  :slots (_order _reward _actions ))

(defmethod hexapod_training::ResultMsg
  (:init
   (&key
    ((:order __order) 0)
    ((:reward __reward) 0.0)
    ((:actions __actions) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _order (round __order))
   (setq _reward (float __reward))
   (setq _actions __actions)
   self)
  (:order
   (&optional __order)
   (if __order (setq _order __order)) _order)
  (:reward
   (&optional __reward)
   (if __reward (setq _reward __reward)) _reward)
  (:actions
   (&optional __actions)
   (if __actions (setq _actions __actions)) _actions)
  (:serialization-length
   ()
   (+
    ;; int8 _order
    1
    ;; float64 _reward
    8
    ;; float64[] _actions
    (* 8    (length _actions)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _order
       (write-byte _order s)
     ;; float64 _reward
       (sys::poke _reward (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64[] _actions
     (write-long (length _actions) s)
     (dotimes (i (length _actions))
       (sys::poke (elt _actions i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _order
     (setq _order (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _order 127) (setq _order (- _order 256)))
   ;; float64 _reward
     (setq _reward (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64[] _actions
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _actions (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _actions i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(setf (get hexapod_training::ResultMsg :md5sum-) "68ca254f3ffbf2a1605af7da24eb91b4")
(setf (get hexapod_training::ResultMsg :datatype-) "hexapod_training/ResultMsg")
(setf (get hexapod_training::ResultMsg :definition-)
      "# ResultMsg.msg
int8 order
float64 reward
float64[] actions
")



(provide :hexapod_training/ResultMsg "68ca254f3ffbf2a1605af7da24eb91b4")


