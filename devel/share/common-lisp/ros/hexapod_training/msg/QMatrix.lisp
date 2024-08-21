; Auto-generated. Do not edit!


(cl:in-package hexapod_training-msg)


;//! \htmlinclude QMatrix.msg.html

(cl:defclass <QMatrix> (roslisp-msg-protocol:ros-message)
  ((elements
    :reader elements
    :initarg :elements
    :type (cl:vector hexapod_training-msg:QMatrixElement)
   :initform (cl:make-array 0 :element-type 'hexapod_training-msg:QMatrixElement :initial-element (cl:make-instance 'hexapod_training-msg:QMatrixElement))))
)

(cl:defclass QMatrix (<QMatrix>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <QMatrix>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'QMatrix)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hexapod_training-msg:<QMatrix> is deprecated: use hexapod_training-msg:QMatrix instead.")))

(cl:ensure-generic-function 'elements-val :lambda-list '(m))
(cl:defmethod elements-val ((m <QMatrix>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hexapod_training-msg:elements-val is deprecated.  Use hexapod_training-msg:elements instead.")
  (elements m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <QMatrix>) ostream)
  "Serializes a message object of type '<QMatrix>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'elements))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'elements))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <QMatrix>) istream)
  "Deserializes a message object of type '<QMatrix>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'elements) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'elements)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'hexapod_training-msg:QMatrixElement))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<QMatrix>)))
  "Returns string type for a message object of type '<QMatrix>"
  "hexapod_training/QMatrix")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'QMatrix)))
  "Returns string type for a message object of type 'QMatrix"
  "hexapod_training/QMatrix")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<QMatrix>)))
  "Returns md5sum for a message object of type '<QMatrix>"
  "fdb7bde857788bc279933d9d650b6350")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'QMatrix)))
  "Returns md5sum for a message object of type 'QMatrix"
  "fdb7bde857788bc279933d9d650b6350")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<QMatrix>)))
  "Returns full string definition for message of type '<QMatrix>"
  (cl:format cl:nil "#QMatrix.msg~%hexapod_training/QMatrixElement[] elements~%~%================================================================================~%MSG: hexapod_training/QMatrixElement~%# QMatrixElement.msg~%hexapod_training/StateActionPair pair~%std_msgs/Float64 reward~%~%================================================================================~%MSG: hexapod_training/StateActionPair~%# StateActionPair.msg~%string state~%int32 action~%================================================================================~%MSG: std_msgs/Float64~%float64 data~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'QMatrix)))
  "Returns full string definition for message of type 'QMatrix"
  (cl:format cl:nil "#QMatrix.msg~%hexapod_training/QMatrixElement[] elements~%~%================================================================================~%MSG: hexapod_training/QMatrixElement~%# QMatrixElement.msg~%hexapod_training/StateActionPair pair~%std_msgs/Float64 reward~%~%================================================================================~%MSG: hexapod_training/StateActionPair~%# StateActionPair.msg~%string state~%int32 action~%================================================================================~%MSG: std_msgs/Float64~%float64 data~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <QMatrix>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'elements) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <QMatrix>))
  "Converts a ROS message object to a list"
  (cl:list 'QMatrix
    (cl:cons ':elements (elements msg))
))
