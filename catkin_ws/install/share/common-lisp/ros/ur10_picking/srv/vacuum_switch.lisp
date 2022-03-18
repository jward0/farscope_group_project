; Auto-generated. Do not edit!


(cl:in-package ur10_picking-srv)


;//! \htmlinclude vacuum_switch-request.msg.html

(cl:defclass <vacuum_switch-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass vacuum_switch-request (<vacuum_switch-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <vacuum_switch-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'vacuum_switch-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ur10_picking-srv:<vacuum_switch-request> is deprecated: use ur10_picking-srv:vacuum_switch-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <vacuum_switch-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ur10_picking-srv:input-val is deprecated.  Use ur10_picking-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <vacuum_switch-request>) ostream)
  "Serializes a message object of type '<vacuum_switch-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'input) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <vacuum_switch-request>) istream)
  "Deserializes a message object of type '<vacuum_switch-request>"
    (cl:setf (cl:slot-value msg 'input) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<vacuum_switch-request>)))
  "Returns string type for a service object of type '<vacuum_switch-request>"
  "ur10_picking/vacuum_switchRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'vacuum_switch-request)))
  "Returns string type for a service object of type 'vacuum_switch-request"
  "ur10_picking/vacuum_switchRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<vacuum_switch-request>)))
  "Returns md5sum for a message object of type '<vacuum_switch-request>"
  "2d82a0d3cd91c1b1995bd03ea0dc40fd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'vacuum_switch-request)))
  "Returns md5sum for a message object of type 'vacuum_switch-request"
  "2d82a0d3cd91c1b1995bd03ea0dc40fd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<vacuum_switch-request>)))
  "Returns full string definition for message of type '<vacuum_switch-request>"
  (cl:format cl:nil "bool input~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'vacuum_switch-request)))
  "Returns full string definition for message of type 'vacuum_switch-request"
  (cl:format cl:nil "bool input~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <vacuum_switch-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <vacuum_switch-request>))
  "Converts a ROS message object to a list"
  (cl:list 'vacuum_switch-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude vacuum_switch-response.msg.html

(cl:defclass <vacuum_switch-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type cl:string
    :initform ""))
)

(cl:defclass vacuum_switch-response (<vacuum_switch-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <vacuum_switch-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'vacuum_switch-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ur10_picking-srv:<vacuum_switch-response> is deprecated: use ur10_picking-srv:vacuum_switch-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <vacuum_switch-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ur10_picking-srv:output-val is deprecated.  Use ur10_picking-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <vacuum_switch-response>) ostream)
  "Serializes a message object of type '<vacuum_switch-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'output))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'output))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <vacuum_switch-response>) istream)
  "Deserializes a message object of type '<vacuum_switch-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'output) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'output) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<vacuum_switch-response>)))
  "Returns string type for a service object of type '<vacuum_switch-response>"
  "ur10_picking/vacuum_switchResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'vacuum_switch-response)))
  "Returns string type for a service object of type 'vacuum_switch-response"
  "ur10_picking/vacuum_switchResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<vacuum_switch-response>)))
  "Returns md5sum for a message object of type '<vacuum_switch-response>"
  "2d82a0d3cd91c1b1995bd03ea0dc40fd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'vacuum_switch-response)))
  "Returns md5sum for a message object of type 'vacuum_switch-response"
  "2d82a0d3cd91c1b1995bd03ea0dc40fd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<vacuum_switch-response>)))
  "Returns full string definition for message of type '<vacuum_switch-response>"
  (cl:format cl:nil "string output~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'vacuum_switch-response)))
  "Returns full string definition for message of type 'vacuum_switch-response"
  (cl:format cl:nil "string output~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <vacuum_switch-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <vacuum_switch-response>))
  "Converts a ROS message object to a list"
  (cl:list 'vacuum_switch-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'vacuum_switch)))
  'vacuum_switch-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'vacuum_switch)))
  'vacuum_switch-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'vacuum_switch)))
  "Returns string type for a service object of type '<vacuum_switch>"
  "ur10_picking/vacuum_switch")