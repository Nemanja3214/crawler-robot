; Auto-generated. Do not edit!


(cl:in-package hexapod_training-msg)


;//! \htmlinclude StateActionPair.msg.html

(cl:defclass <StateActionPair> (roslisp-msg-protocol:ros-message)
  ((state
    :reader state
    :initarg :state
    :type cl:string
    :initform "")
   (action
    :reader action
    :initarg :action
    :type cl:integer
    :initform 0))
)

(cl:defclass StateActionPair (<StateActionPair>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <StateActionPair>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'StateActionPair)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hexapod_training-msg:<StateActionPair> is deprecated: use hexapod_training-msg:StateActionPair instead.")))

(cl:ensure-generic-function 'state-val :lambda-list '(m))
(cl:defmethod state-val ((m <StateActionPair>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hexapod_training-msg:state-val is deprecated.  Use hexapod_training-msg:state instead.")
  (state m))

(cl:ensure-generic-function 'action-val :lambda-list '(m))
(cl:defmethod action-val ((m <StateActionPair>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hexapod_training-msg:action-val is deprecated.  Use hexapod_training-msg:action instead.")
  (action m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <StateActionPair>) ostream)
  "Serializes a message object of type '<StateActionPair>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'state))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'state))
  (cl:let* ((signed (cl:slot-value msg 'action)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <StateActionPair>) istream)
  "Deserializes a message object of type '<StateActionPair>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'state) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'state) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'action) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<StateActionPair>)))
  "Returns string type for a message object of type '<StateActionPair>"
  "hexapod_training/StateActionPair")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StateActionPair)))
  "Returns string type for a message object of type 'StateActionPair"
  "hexapod_training/StateActionPair")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<StateActionPair>)))
  "Returns md5sum for a message object of type '<StateActionPair>"
  "ea8f4a97a7ca851b7f7f92f5d00b7f93")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'StateActionPair)))
  "Returns md5sum for a message object of type 'StateActionPair"
  "ea8f4a97a7ca851b7f7f92f5d00b7f93")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<StateActionPair>)))
  "Returns full string definition for message of type '<StateActionPair>"
  (cl:format cl:nil "# StateActionPair.msg~%string state~%int32 action~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'StateActionPair)))
  "Returns full string definition for message of type 'StateActionPair"
  (cl:format cl:nil "# StateActionPair.msg~%string state~%int32 action~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <StateActionPair>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'state))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <StateActionPair>))
  "Converts a ROS message object to a list"
  (cl:list 'StateActionPair
    (cl:cons ':state (state msg))
    (cl:cons ':action (action msg))
))
