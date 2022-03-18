; Auto-generated. Do not edit!


(cl:in-package ur10_picking-msg)


;//! \htmlinclude poseMessage.msg.html

(cl:defclass <poseMessage> (roslisp-msg-protocol:ros-message)
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

(cl:defclass poseMessage (<poseMessage>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <poseMessage>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'poseMessage)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ur10_picking-msg:<poseMessage> is deprecated: use ur10_picking-msg:poseMessage instead.")))

(cl:ensure-generic-function 'incremental-val :lambda-list '(m))
(cl:defmethod incremental-val ((m <poseMessage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ur10_picking-msg:incremental-val is deprecated.  Use ur10_picking-msg:incremental instead.")
  (incremental m))

(cl:ensure-generic-function 'pose-val :lambda-list '(m))
(cl:defmethod pose-val ((m <poseMessage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ur10_picking-msg:pose-val is deprecated.  Use ur10_picking-msg:pose instead.")
  (pose m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <poseMessage>) ostream)
  "Serializes a message object of type '<poseMessage>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'incremental) 1 0)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pose) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <poseMessage>) istream)
  "Deserializes a message object of type '<poseMessage>"
    (cl:setf (cl:slot-value msg 'incremental) (cl:not (cl:zerop (cl:read-byte istream))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pose) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<poseMessage>)))
  "Returns string type for a message object of type '<poseMessage>"
  "ur10_picking/poseMessage")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'poseMessage)))
  "Returns string type for a message object of type 'poseMessage"
  "ur10_picking/poseMessage")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<poseMessage>)))
  "Returns md5sum for a message object of type '<poseMessage>"
  "a78b0cf13928e95a7f4fb13c941b8e3e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'poseMessage)))
  "Returns md5sum for a message object of type 'poseMessage"
  "a78b0cf13928e95a7f4fb13c941b8e3e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<poseMessage>)))
  "Returns full string definition for message of type '<poseMessage>"
  (cl:format cl:nil "bool incremental~%geometry_msgs/Pose pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'poseMessage)))
  "Returns full string definition for message of type 'poseMessage"
  (cl:format cl:nil "bool incremental~%geometry_msgs/Pose pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <poseMessage>))
  (cl:+ 0
     1
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pose))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <poseMessage>))
  "Converts a ROS message object to a list"
  (cl:list 'poseMessage
    (cl:cons ':incremental (incremental msg))
    (cl:cons ':pose (pose msg))
))
