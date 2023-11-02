;; Auto-generated. Do not edit!


(when (boundp 'trans::ecu)
  (if (not (find-package "TRANS"))
    (make-package "TRANS"))
  (shadow 'ecu (find-package "TRANS")))
(unless (find-package "TRANS::ECU")
  (make-package "TRANS::ECU"))

(in-package "ROS")
;;//! \htmlinclude ecu.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(intern "*SHIFT_UNKNOWN*" (find-package "TRANS::ECU"))
(shadow '*SHIFT_UNKNOWN* (find-package "TRANS::ECU"))
(defconstant trans::ecu::*SHIFT_UNKNOWN* 0)
(intern "*SHIFT_D*" (find-package "TRANS::ECU"))
(shadow '*SHIFT_D* (find-package "TRANS::ECU"))
(defconstant trans::ecu::*SHIFT_D* 1)
(intern "*SHIFT_N*" (find-package "TRANS::ECU"))
(shadow '*SHIFT_N* (find-package "TRANS::ECU"))
(defconstant trans::ecu::*SHIFT_N* 2)
(intern "*SHIFT_R*" (find-package "TRANS::ECU"))
(shadow '*SHIFT_R* (find-package "TRANS::ECU"))
(defconstant trans::ecu::*SHIFT_R* 3)
(intern "*SHIFT_T*" (find-package "TRANS::ECU"))
(shadow '*SHIFT_T* (find-package "TRANS::ECU"))
(defconstant trans::ecu::*SHIFT_T* 9)
(defclass trans::ecu
  :super ros::object
  :slots (_header _motor _steer _brake _cur_speed _speed_ratio _shift ))

(defmethod trans::ecu
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:motor __motor) 0.0)
    ((:steer __steer) 0.0)
    ((:brake __brake) nil)
    ((:cur_speed __cur_speed) 0.0)
    ((:speed_ratio __speed_ratio) 0.0)
    ((:shift __shift) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _motor (float __motor))
   (setq _steer (float __steer))
   (setq _brake __brake)
   (setq _cur_speed (float __cur_speed))
   (setq _speed_ratio (float __speed_ratio))
   (setq _shift (round __shift))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:motor
   (&optional __motor)
   (if __motor (setq _motor __motor)) _motor)
  (:steer
   (&optional __steer)
   (if __steer (setq _steer __steer)) _steer)
  (:brake
   (&optional __brake)
   (if __brake (setq _brake __brake)) _brake)
  (:cur_speed
   (&optional __cur_speed)
   (if __cur_speed (setq _cur_speed __cur_speed)) _cur_speed)
  (:speed_ratio
   (&optional __speed_ratio)
   (if __speed_ratio (setq _speed_ratio __speed_ratio)) _speed_ratio)
  (:shift
   (&optional __shift)
   (if __shift (setq _shift __shift)) _shift)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float32 _motor
    4
    ;; float32 _steer
    4
    ;; bool _brake
    1
    ;; float32 _cur_speed
    4
    ;; float32 _speed_ratio
    4
    ;; uint8 _shift
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float32 _motor
       (sys::poke _motor (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _steer
       (sys::poke _steer (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; bool _brake
       (if _brake (write-byte -1 s) (write-byte 0 s))
     ;; float32 _cur_speed
       (sys::poke _cur_speed (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _speed_ratio
       (sys::poke _speed_ratio (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; uint8 _shift
       (write-byte _shift s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float32 _motor
     (setq _motor (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _steer
     (setq _steer (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; bool _brake
     (setq _brake (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float32 _cur_speed
     (setq _cur_speed (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _speed_ratio
     (setq _speed_ratio (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; uint8 _shift
     (setq _shift (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get trans::ecu :md5sum-) "59bf7eba7675d619ecb8c5ad9e66f08b")
(setf (get trans::ecu :datatype-) "trans/ecu")
(setf (get trans::ecu :definition-)
      "Header header

float32 motor # 目标速度
float32 steer # 转向
bool brake # 紧急停车
float32 cur_speed # 当前速度
float32 speed_ratio # 控制速度比率的

uint8 shift # 档位
uint8 SHIFT_UNKNOWN = 0 
uint8 SHIFT_D = 1 #前进档位
uint8 SHIFT_N = 2 #停止档位
uint8 SHIFT_R = 3 #后退档位
uint8 SHIFT_T = 9 #遥控

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

")



(provide :trans/ecu "59bf7eba7675d619ecb8c5ad9e66f08b")


