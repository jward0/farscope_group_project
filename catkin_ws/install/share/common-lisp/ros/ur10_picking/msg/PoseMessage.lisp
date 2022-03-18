; Auto-generated. Do not edit!


(cl:in-package ur10_picking-msg)


;//! \htmlinclude PoseMessage.msg.html

(cl:defclass <PoseMessage> (roslisp-msg-protocol:ros-message)
  ((incremental
    :reader incremental
    :initarg :incremental
    :type cl:boolean
    :initform cl:nil)
   (pose
    :reader pose
    :initarg :pose
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose)))
)

(cl:defclass PoseMessage (<PoseMessage>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PoseMessage>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PoseMessage)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ur10_picking-msg:<PoseMessage> is deprecated: use ur10_picking-msg:PoseMessage instead.")))

(cl:ensure-generic-function 'incremental-val :lambda-list '(m))
(cl:defmethod incremental-val ((m <PoseMessage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ur10_picking-msg:incremental-val is deprecated.  Use ur10_picking-msg:incremental instead.")
  (incremental m))

(cl:ensure-generic-function 'pose-val :lambda-list '(m))
(cl:defmethod pose-val ((m <PoseMessage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ur10_picking-msg:pose-val is deprecated.  Use ur10_picking-msg:pose instead.")
  (pose m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PoseMessage>) ostream)
  "Serializes a message object of type '<PoseMessage>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'incremental) 1 0)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pose) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PoseMessage>) istream)
  "Deserializes a message object of type '<PoseMessage>"
    (cl:setf (cl:slot-value msg 'incremental) (cl:not (cl:zerop (cl:read-byte istream))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pose) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PoseMessage>)))
  "Returns string type for a message object of type '<PoseMessage>"
  "ur10_picking/PoseMessage")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PoseMessage)))
  "Returns string type for a message object of type 'PoseMessage"
  "ur10_picking/PoseMessage")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PoseMessage>)))
  "Returns md5sum for a message object of type '<PoseMessage>"
  "a78b0cf13928e95a7f4fb13c941b8e3e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PoseMessage)))
  "Returns md5sum for a message object of type 'PoseMessage"
  "a78b0cf13928e95a7f4fb13c941b8e3e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PoseMessage>)))
  "Returns full string definition for message of type '<PoseMessage>"
  (cl:format cl:nil "bool incremental~%geometry_msgs/Pose pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PoseMessage)))
  "Returns full string definition for message of type 'PoseMessage"
  (cl:format cl:nil "bool incremental~%geometry_msgs/Pose pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PoseMessage>))
  (cl:+ 0
     1
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pose))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PoseMessage>))
  "Converts a ROS message object to a list"
  (cl:list 'PoseMessage
    (cl:cons ':incremental (incremental msg))
    (cl:cons ':pose (pose msg))
))
