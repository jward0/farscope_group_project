;; Auto-generated. Do not edit!


(when (boundp 'ur10_picking::vacuum_calibration)
  (if (not (find-package "UR10_PICKING"))
    (make-package "UR10_PICKING"))
  (shadow 'vacuum_calibration (find-package "UR10_PICKING")))
(unless (find-package "UR10_PICKING::VACUUM_CALIBRATION")
  (make-package "UR10_PICKING::VACUUM_CALIBRATION"))
(unless (find-package "UR10_PICKING::VACUUM_CALIBRATIONREQUEST")
  (make-package "UR10_PICKING::VACUUM_CALIBRATIONREQUEST"))
(unless (find-package "UR10_PICKING::VACUUM_CALIBRATIONRESPONSE")
  (make-package "UR10_PICKING::VACUUM_CALIBRATIONRESPONSE"))

(in-package "ROS")





(defclass ur10_picking::vacuum_calibrationRequest
  :super ros::object
  :slots (_input ))

(defmethod ur10_picking::vacuum_calibrationRequest
  (:init
   (&key
    ((:input __input) "")
    )
   (send-super :init)
   (setq _input (string __input))
   self)
  (:input
   (&optional __input)
   (if __input (setq _input __input)) _input)
  (:serialization-length
   ()
   (+
    ;; string _input
    4 (length _input)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _input
       (write-long (length _input) s) (princ _input s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _input
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _input (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass ur10_picking::vacuum_calibrationResponse
  :super ros::object
  :slots (_output ))

(defmethod ur10_picking::vacuum_calibrationResponse
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

(defclass ur10_picking::vacuum_calibration
  :super ros::object
  :slots ())

(setf (get ur10_picking::vacuum_calibration :md5sum-) "c63e85f503b805d84df783e71c6bb0d2")
(setf (get ur10_picking::vacuum_calibration :datatype-) "ur10_picking/vacuum_calibration")
(setf (get ur10_picking::vacuum_calibration :request) ur10_picking::vacuum_calibrationRequest)
(setf (get ur10_picking::vacuum_calibration :response) ur10_picking::vacuum_calibrationResponse)

(defmethod ur10_picking::vacuum_calibrationRequest
  (:response () (instance ur10_picking::vacuum_calibrationResponse :init)))

(setf (get ur10_picking::vacuum_calibrationRequest :md5sum-) "c63e85f503b805d84df783e71c6bb0d2")
(setf (get ur10_picking::vacuum_calibrationRequest :datatype-) "ur10_picking/vacuum_calibrationRequest")
(setf (get ur10_picking::vacuum_calibrationRequest :definition-)
      "string input
---
string output

")

(setf (get ur10_picking::vacuum_calibrationResponse :md5sum-) "c63e85f503b805d84df783e71c6bb0d2")
(setf (get ur10_picking::vacuum_calibrationResponse :datatype-) "ur10_picking/vacuum_calibrationResponse")
(setf (get ur10_picking::vacuum_calibrationResponse :definition-)
      "string input
---
string output

")



(provide :ur10_picking/vacuum_calibration "c63e85f503b805d84df783e71c6bb0d2")


