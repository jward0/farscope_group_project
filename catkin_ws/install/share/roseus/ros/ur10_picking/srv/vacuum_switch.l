;; Auto-generated. Do not edit!


(when (boundp 'ur10_picking::vacuum_switch)
  (if (not (find-package "UR10_PICKING"))
    (make-package "UR10_PICKING"))
  (shadow 'vacuum_switch (find-package "UR10_PICKING")))
(unless (find-package "UR10_PICKING::VACUUM_SWITCH")
  (make-package "UR10_PICKING::VACUUM_SWITCH"))
(unless (find-package "UR10_PICKING::VACUUM_SWITCHREQUEST")
  (make-package "UR10_PICKING::VACUUM_SWITCHREQUEST"))
(unless (find-package "UR10_PICKING::VACUUM_SWITCHRESPONSE")
  (make-package "UR10_PICKING::VACUUM_SWITCHRESPONSE"))

(in-package "ROS")





(defclass ur10_picking::vacuum_switchRequest
  :super ros::object
  :slots (_input ))

(defmethod ur10_picking::vacuum_switchRequest
  (:init
   (&key
    ((:input __input) nil)
    )
   (send-super :init)
   (setq _input __input)
   self)
  (:input
   (&optional __input)
   (if __input (setq _input __input)) _input)
  (:serialization-length
   ()
   (+
    ;; bool _input
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _input
       (if _input (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _input
     (setq _input (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass ur10_picking::vacuum_switchResponse
  :super ros::object
  :slots (_output ))

(defmethod ur10_picking::vacuum_switchResponse
  (:init
   (&key
    ((:output __output) "")
    )
   (send-super :init)
   (setq _output (string __output))
   self)
  (:output
   (&optional __output)
   (if __output (setq _output __output)) _output)
  (:serialization-length
   ()
   (+
    ;; string _output
    4 (length _output)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _output
       (write-long (length _output) s) (princ _output s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _output
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _output (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass ur10_picking::vacuum_switch
  :super ros::object
  :slots ())

(setf (get ur10_picking::vacuum_switch :md5sum-) "2d82a0d3cd91c1b1995bd03ea0dc40fd")
(setf (get ur10_picking::vacuum_switch :datatype-) "ur10_picking/vacuum_switch")
(setf (get ur10_picking::vacuum_switch :request) ur10_picking::vacuum_switchRequest)
(setf (get ur10_picking::vacuum_switch :response) ur10_picking::vacuum_switchResponse)

(defmethod ur10_picking::vacuum_switchRequest
  (:response () (instance ur10_picking::vacuum_switchResponse :init)))

(setf (get ur10_picking::vacuum_switchRequest :md5sum-) "2d82a0d3cd91c1b1995bd03ea0dc40fd")
(setf (get ur10_picking::vacuum_switchRequest :datatype-) "ur10_picking/vacuum_switchRequest")
(setf (get ur10_picking::vacuum_switchRequest :definition-)
      "bool input
---
string output

")

(setf (get ur10_picking::vacuum_switchResponse :md5sum-) "2d82a0d3cd91c1b1995bd03ea0dc40fd")
(setf (get ur10_picking::vacuum_switchResponse :datatype-) "ur10_picking/vacuum_switchResponse")
(setf (get ur10_picking::vacuum_switchResponse :definition-)
      "bool input
---
string output

")



(provide :ur10_picking/vacuum_switch "2d82a0d3cd91c1b1995bd03ea0dc40fd")


