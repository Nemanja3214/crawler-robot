; Auto-generated. Do not edit!


(cl:in-package hexapod_training-msg)


;//! \htmlinclude QMatrixElement.msg.html

(cl:defclass <QMatrixElement> (roslisp-msg-protocol:ros-message)
  ((pair
    :reader pair
    :initarg :pair
    :type hexapod_training-msg:StateActionPair
    :initform (cl:make-instance 'hexapod_training-msg:StateActionPair))
   (reward
    :reader reward
    :initarg :reward
    :type std_msgs-msg:Float64
    :initform (cl:make-instance 'std_msgs-msg:Float64)))
)

(cl:defclass QMatrixElement (<QMatrixElement>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <QMatrixElement>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'QMatrixElement)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hexapod_training-msg:<QMatrixElement> is deprecated: use hexapod_training-msg:QMatrixElement instead.")))

(cl:ensure-generic-function 'pair-val :lambda-list '(m))
(cl:defmethod pair-val ((m <QMatrixElement>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hexapod_training-msg:pair-val is deprecated.  Use hexapod_training-msg:pair instead.")
  (pair m))

(cl:ensure-generic-function 'reward-val :lambda-list '(m))
(cl:defmethod reward-val ((m <QMatrixElement>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hexapod_training-msg:reward-val is deprecated.  Use hexapod_training-msg:reward instead.")
  (reward m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <QMatrixElement>) ostream)
  "Serializes a message object of type '<QMatrixElement>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pair) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'reward) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <QMatrixElement>) istream)
  "Deserializes a message object of type '<QMatrixElement>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pair) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'reward) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<QMatrixElement>)))
  "Returns string type for a message object of type '<QMatrixElement>"
  "hexapod_training/QMatrixElement")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'QMatrixElement)))
  "Returns string type for a message object of type 'QMatrixElement"
  "hexapod_training/QMatrixElement")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<QMatrixElement>)))
  "Returns md5sum for a message object of type '<QMatrixElement>"
  "39b6677d577d14397b6281d04d889aab")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'QMatrixElement)))
  "Returns md5sum for a message object of type 'QMatrixElement"
  "39b6677d577d14397b6281d04d889aab")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<QMatrixElement>)))
  "Returns full string definition for message of type '<QMatrixElement>"
  (cl:format cl:nil "# QMatrixElement.msg~%hexapod_training/StateActionPair pair~%std_msgs/Float64 reward~%~%================================================================================~%MSG: hexapod_training/StateActionPair~%# StateActionPair.msg~%string state~%int32 action~%================================================================================~%MSG: std_msgs/Float64~%float64 data~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'QMatrixElement)))
  "Returns full string definition for message of type 'QMatrixElement"
  (cl:format cl:nil "# QMatrixElement.msg~%hexapod_training/StateActionPair pair~%std_msgs/Float64 reward~%~%================================================================================~%MSG: hexapod_training/StateActionPair~%# StateActionPair.msg~%string state~%int32 action~%================================================================================~%MSG: std_msgs/Float64~%float64 data~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <QMatrixElement>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pair))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'reward))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <QMatrixElement>))
  "Converts a ROS message object to a list"
  (cl:list 'QMatrixElement
    (cl:cons ':pair (pair msg))
    (cl:cons ':reward (reward msg))
))
