; Auto-generated. Do not edit!


(cl:in-package hexapod_training-msg)


;//! \htmlinclude ResultMsg.msg.html

(cl:defclass <ResultMsg> (roslisp-msg-protocol:ros-message)
  ((order
    :reader order
    :initarg :order
    :type cl:fixnum
    :initform 0)
   (reward
    :reader reward
    :initarg :reward
    :type cl:float
    :initform 0.0)
   (actions
    :reader actions
    :initarg :actions
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass ResultMsg (<ResultMsg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ResultMsg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ResultMsg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hexapod_training-msg:<ResultMsg> is deprecated: use hexapod_training-msg:ResultMsg instead.")))

(cl:ensure-generic-function 'order-val :lambda-list '(m))
(cl:defmethod order-val ((m <ResultMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hexapod_training-msg:order-val is deprecated.  Use hexapod_training-msg:order instead.")
  (order m))

(cl:ensure-generic-function 'reward-val :lambda-list '(m))
(cl:defmethod reward-val ((m <ResultMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hexapod_training-msg:reward-val is deprecated.  Use hexapod_training-msg:reward instead.")
  (reward m))

(cl:ensure-generic-function 'actions-val :lambda-list '(m))
(cl:defmethod actions-val ((m <ResultMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hexapod_training-msg:actions-val is deprecated.  Use hexapod_training-msg:actions instead.")
  (actions m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ResultMsg>) ostream)
  "Serializes a message object of type '<ResultMsg>"
  (cl:let* ((signed (cl:slot-value msg 'order)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'reward))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'actions))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'actions))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ResultMsg>) istream)
  "Deserializes a message object of type '<ResultMsg>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'order) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'reward) (roslisp-utils:decode-double-float-bits bits)))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'actions) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'actions)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ResultMsg>)))
  "Returns string type for a message object of type '<ResultMsg>"
  "hexapod_training/ResultMsg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ResultMsg)))
  "Returns string type for a message object of type 'ResultMsg"
  "hexapod_training/ResultMsg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ResultMsg>)))
  "Returns md5sum for a message object of type '<ResultMsg>"
  "68ca254f3ffbf2a1605af7da24eb91b4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ResultMsg)))
  "Returns md5sum for a message object of type 'ResultMsg"
  "68ca254f3ffbf2a1605af7da24eb91b4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ResultMsg>)))
  "Returns full string definition for message of type '<ResultMsg>"
  (cl:format cl:nil "# ResultMsg.msg~%int8 order~%float64 reward~%float64[] actions~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ResultMsg)))
  "Returns full string definition for message of type 'ResultMsg"
  (cl:format cl:nil "# ResultMsg.msg~%int8 order~%float64 reward~%float64[] actions~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ResultMsg>))
  (cl:+ 0
     1
     8
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'actions) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ResultMsg>))
  "Converts a ROS message object to a list"
  (cl:list 'ResultMsg
    (cl:cons ':order (order msg))
    (cl:cons ':reward (reward msg))
    (cl:cons ':actions (actions msg))
))
