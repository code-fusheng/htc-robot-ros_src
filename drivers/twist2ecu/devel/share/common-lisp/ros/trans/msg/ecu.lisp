; Auto-generated. Do not edit!


(cl:in-package trans-msg)


;//! \htmlinclude ecu.msg.html

(cl:defclass <ecu> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (motor
    :reader motor
    :initarg :motor
    :type cl:float
    :initform 0.0)
   (steer
    :reader steer
    :initarg :steer
    :type cl:float
    :initform 0.0)
   (brake
    :reader brake
    :initarg :brake
    :type cl:boolean
    :initform cl:nil)
   (cur_speed
    :reader cur_speed
    :initarg :cur_speed
    :type cl:float
    :initform 0.0)
   (speed_ratio
    :reader speed_ratio
    :initarg :speed_ratio
    :type cl:float
    :initform 0.0)
   (shift
    :reader shift
    :initarg :shift
    :type cl:fixnum
    :initform 0))
)

(cl:defclass ecu (<ecu>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ecu>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ecu)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name trans-msg:<ecu> is deprecated: use trans-msg:ecu instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <ecu>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader trans-msg:header-val is deprecated.  Use trans-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'motor-val :lambda-list '(m))
(cl:defmethod motor-val ((m <ecu>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader trans-msg:motor-val is deprecated.  Use trans-msg:motor instead.")
  (motor m))

(cl:ensure-generic-function 'steer-val :lambda-list '(m))
(cl:defmethod steer-val ((m <ecu>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader trans-msg:steer-val is deprecated.  Use trans-msg:steer instead.")
  (steer m))

(cl:ensure-generic-function 'brake-val :lambda-list '(m))
(cl:defmethod brake-val ((m <ecu>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader trans-msg:brake-val is deprecated.  Use trans-msg:brake instead.")
  (brake m))

(cl:ensure-generic-function 'cur_speed-val :lambda-list '(m))
(cl:defmethod cur_speed-val ((m <ecu>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader trans-msg:cur_speed-val is deprecated.  Use trans-msg:cur_speed instead.")
  (cur_speed m))

(cl:ensure-generic-function 'speed_ratio-val :lambda-list '(m))
(cl:defmethod speed_ratio-val ((m <ecu>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader trans-msg:speed_ratio-val is deprecated.  Use trans-msg:speed_ratio instead.")
  (speed_ratio m))

(cl:ensure-generic-function 'shift-val :lambda-list '(m))
(cl:defmethod shift-val ((m <ecu>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader trans-msg:shift-val is deprecated.  Use trans-msg:shift instead.")
  (shift m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<ecu>)))
    "Constants for message type '<ecu>"
  '((:SHIFT_UNKNOWN . 0)
    (:SHIFT_D . 1)
    (:SHIFT_N . 2)
    (:SHIFT_R . 3)
    (:SHIFT_T . 9))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'ecu)))
    "Constants for message type 'ecu"
  '((:SHIFT_UNKNOWN . 0)
    (:SHIFT_D . 1)
    (:SHIFT_N . 2)
    (:SHIFT_R . 3)
    (:SHIFT_T . 9))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ecu>) ostream)
  "Serializes a message object of type '<ecu>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'motor))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'steer))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'brake) 1 0)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'cur_speed))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'speed_ratio))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'shift)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ecu>) istream)
  "Deserializes a message object of type '<ecu>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'motor) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'steer) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:slot-value msg 'brake) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'cur_speed) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'speed_ratio) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'shift)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ecu>)))
  "Returns string type for a message object of type '<ecu>"
  "trans/ecu")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ecu)))
  "Returns string type for a message object of type 'ecu"
  "trans/ecu")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ecu>)))
  "Returns md5sum for a message object of type '<ecu>"
  "59bf7eba7675d619ecb8c5ad9e66f08b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ecu)))
  "Returns md5sum for a message object of type 'ecu"
  "59bf7eba7675d619ecb8c5ad9e66f08b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ecu>)))
  "Returns full string definition for message of type '<ecu>"
  (cl:format cl:nil "Header header~%~%float32 motor # 目标速度~%float32 steer # 转向~%bool brake # 紧急停车~%float32 cur_speed # 当前速度~%float32 speed_ratio # 控制速度比率的~%~%uint8 shift # 档位~%uint8 SHIFT_UNKNOWN = 0 ~%uint8 SHIFT_D = 1 #前进档位~%uint8 SHIFT_N = 2 #停止档位~%uint8 SHIFT_R = 3 #后退档位~%uint8 SHIFT_T = 9 #遥控~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ecu)))
  "Returns full string definition for message of type 'ecu"
  (cl:format cl:nil "Header header~%~%float32 motor # 目标速度~%float32 steer # 转向~%bool brake # 紧急停车~%float32 cur_speed # 当前速度~%float32 speed_ratio # 控制速度比率的~%~%uint8 shift # 档位~%uint8 SHIFT_UNKNOWN = 0 ~%uint8 SHIFT_D = 1 #前进档位~%uint8 SHIFT_N = 2 #停止档位~%uint8 SHIFT_R = 3 #后退档位~%uint8 SHIFT_T = 9 #遥控~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ecu>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     4
     1
     4
     4
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ecu>))
  "Converts a ROS message object to a list"
  (cl:list 'ecu
    (cl:cons ':header (header msg))
    (cl:cons ':motor (motor msg))
    (cl:cons ':steer (steer msg))
    (cl:cons ':brake (brake msg))
    (cl:cons ':cur_speed (cur_speed msg))
    (cl:cons ':speed_ratio (speed_ratio msg))
    (cl:cons ':shift (shift msg))
))
