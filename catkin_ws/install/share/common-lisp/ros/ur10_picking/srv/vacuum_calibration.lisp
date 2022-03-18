; Auto-generated. Do not edit!


(cl:in-package ur10_picking-srv)


;//! \htmlinclude vacuum_calibration-request.msg.html

(cl:defclass <vacuum_calibration-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type cl:string
    :initform ""))
)

(cl:defclass vacuum_calibration-request (<vacuum_calibration-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <vacuum_calibration-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'vacuum_calibration-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ur10_picking-srv:<vacuum_calibration-request> is deprecated: use ur10_picking-srv:vacuum_calibration-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <vacuum_calibration-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ur10_picking-srv:input-val is deprecated.  Use ur10_picking-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <vacuum_calibration-request>) ostream)
  "Serializes a message object of type '<vacuum_calibration-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'input))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'input))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <vacuum_calibration-request>) istream)
  "Deserializes a message object of type '<vacuum_calibration-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'input) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'input) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<vacuum_calibration-request>)))
  "Returns string type for a service object of type '<vacuum_calibration-request>"
  "ur10_picking/vacuum_calibrationRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'vacuum_calibration-request)))
  "Returns string type for a service object of type 'vacuum_calibration-request"
  "ur10_picking/vacuum_calibrationRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<vacuum_calibration-request>)))
  "Returns md5sum for a message object of type '<vacuum_calibration-request>"
  "c63e85f503b805d84df783e71c6bb0d2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'vacuum_calibration-request)))
  "Returns md5sum for a message object of type 'vacuum_calibration-request"
  "c63e85f503b805d84df783e71c6bb0d2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<vacuum_calibration-request>)))
  "Returns full string definition for message of type '<vacuum_calibration-request>"
  (cl:format cl:nil "string input~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'vacuum_calibration-request)))
  "Returns full string definition for message of type 'vacuum_calibration-request"
  (cl:format cl:nil "string input~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <vacuum_calibration-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <vacuum_calibration-request>))
  "Converts a ROS message object to a list"
  (cl:list 'vacuum_calibration-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude vacuum_calibration-response.msg.html

(cl:defclass <vacuum_calibration-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type cl:string
    :initform ""))
)

(cl:defclass vacuum_calibration-response (<vacuum_calibration-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <vacuum_calibration-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'vacuum_calibration-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ur10_picking-srv:<vacuum_calibration-response> is deprecated: use ur10_picking-srv:vacuum_calibration-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <vacuum_calibration-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ur10_picking-srv:output-val is deprecated.  Use ur10_picking-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <vacuum_calibration-response>) ostream)
  "Serializes a message object of type '<vacuum_calibration-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'output))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'output))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <vacuum_calibration-response>) istream)
  "Deserializes a message object of type '<vacuum_calibration-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<vacuum_calibration-response>)))
  "Returns string type for a service object of type '<vacuum_calibration-response>"
  "ur10_picking/vacuum_calibrationResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'vacuum_calibration-response)))
  "Returns string type for a service object of type 'vacuum_calibration-response"
  "ur10_picking/vacuum_calibrationResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<vacuum_calibration-response>)))
  "Returns md5sum for a message object of type '<vacuum_calibration-response>"
  "c63e85f503b805d84df783e71c6bb0d2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'vacuum_calibration-response)))
  "Returns md5sum for a message object of type 'vacuum_calibration-response"
  "c63e85f503b805d84df783e71c6bb0d2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<vacuum_calibration-response>)))
  "Returns full string definition for message of type '<vacuum_calibration-response>"
  (cl:format cl:nil "string output~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'vacuum_calibration-response)))
  "Returns full string definition for message of type 'vacuum_calibration-response"
  (cl:format cl:nil "string output~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <vacuum_calibration-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <vacuum_calibration-response>))
  "Converts a ROS message object to a list"
  (cl:list 'vacuum_calibration-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'vacuum_calibration)))
  'vacuum_calibration-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'vacuum_calibration)))
  'vacuum_calibration-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'vacuum_calibration)))
  "Returns string type for a service object of type '<vacuum_calibration>"
  "ur10_picking/vacuum_calibration")