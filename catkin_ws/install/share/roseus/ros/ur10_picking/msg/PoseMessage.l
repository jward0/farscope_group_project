;; Auto-generated. Do not edit!


(when (boundp 'ur10_picking::PoseMessage)
  (if (not (find-package "UR10_PICKING"))
    (make-package "UR10_PICKING"))
  (shadow 'PoseMessage (find-package "UR10_PICKING")))
(unless (find-package "UR10_PICKING::POSEMESSAGE")
  (make-package "UR10_PICKING::POSEMESSAGE"))

(in-package "ROS")
;;//! \htmlinclude PoseMessage.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass ur10_picking::PoseMessage
  :super ros::object
  :slots (_incremental _pose ))

(defmethod ur10_picking::PoseMessage
  (:init
   (&key
    ((:incremental __incremental) nil)
    ((:pose __pose) (instance geometry_msgs::Pose :init))
    )
   (send-super :init)
   (setq _incremental __incremental)
   (setq _pose __pose)
   self)
  (:incremental
   (&optional __incremental)
   (if __incremental (setq _incremental __incremental)) _incremental)
  (:pose
   (&rest __pose)
   (if (keywordp (car __pose))
       (send* _pose __pose)
     (progn
       (if __pose (setq _pose (car __pose)))
       _pose)))
  (:serialization-length
   ()
   (+
    ;; bool _incremental
    1
    ;; geometry_msgs/Pose _pose
    (send _pose :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _incremental
       (if _incremental (write-byte -1 s) (write-byte 0 s))
     ;; geometry_msgs/Pose _pose
       (send _pose :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _incremental
     (setq _incremental (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; geometry_msgs/Pose _pose
     (send _pose :deserialize buf ptr-) (incf ptr- (send _pose :serialization-length))
   ;;
   self)
  )

(setf (get ur10_picking::PoseMessage :md5sum-) "a78b0cf13928e95a7f4fb13c941b8e3e")
(setf (get ur10_picking::PoseMessage :datatype-) "ur10_picking/PoseMessage")
(setf (get ur10_picking::PoseMessage :definition-)
      "bool incremental
geometry_msgs/Pose pose

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

")



(provide :ur10_picking/PoseMessage "a78b0cf13928e95a7f4fb13c941b8e3e")


