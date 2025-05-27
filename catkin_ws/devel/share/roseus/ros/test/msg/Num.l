;; Auto-generated. Do not edit!


(when (boundp 'test::Num)
  (if (not (find-package "TEST"))
    (make-package "TEST"))
  (shadow 'Num (find-package "TEST")))
(unless (find-package "TEST::NUM")
  (make-package "TEST::NUM"))

(in-package "ROS")
;;//! \htmlinclude Num.msg.html


(defclass test::Num
  :super ros::object
  :slots ())

(defmethod test::Num
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(setf (get test::Num :md5sum-) "d41d8cd98f00b204e9800998ecf8427e")
(setf (get test::Num :datatype-) "test/Num")
(setf (get test::Num :definition-)
      "
")



(provide :test/Num "d41d8cd98f00b204e9800998ecf8427e")


