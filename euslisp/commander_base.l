(ros::load-ros-manifest "roseus")
(ros::roseus-add-msgs "mohou_ros")

(defun assert-isinstance (instance instance_type)
  (unless (derivedp instance instance_type) (error "")))

(defun assert-listp (instance)
  (unless (listp instance) (error "not list")))

(defun vec-to-list (vec)
  (let ((lst-ret nil))
    (dotimes (i (length vec))
      (push (aref vec i) lst-ret))
    (reverse lst-ret)))

(defun joint-name-to-symbol (joint-name)
  (read-from-string (concatenate string ":" joint-name)))
  
(defun standard-to-eus-unut-conversion (joint-object angle)
  ;; precondition validation
  (assert-isinstance joint-object joint)

  ;; main
  (when (derivedp joint-object rotational-joint)
    (return-from standard-to-eus-unut-conversion (rad2deg angle)))
  (when (derivedp joint-object linear-joint)
    (return-from standard-to-eus-unut-conversion (* 1000 angle)))
  (assert nil "the joint type is not supported")
)

(defun convert-joint-angle-list (robot joint-name-list stdunit-angle-list)
  ;; precondition validation
  (assert-isinstance robot robot-model)
  (assert-listp joint-name-list)
  (assert-listp stdunit-angle-list)

  ;; main
  (unless (eq (length joint-name-list) (length stdunit-angle-list)) (error "length must match"))
  (let ((n-joint (length joint-name-list))
        (ret-angle-list '()))
    (dotimes (i n-joint)
      (let* (
             (joint-name (nth i joint-name-list))
             (angle-stdunit (nth i stdunit-angle-list))
             (joint-symbol (joint-name-to-symbol joint-name))
             (joint-object (send robot joint-symbol))
             )
        (push (standard-to-eus-unut-conversion joint-object angle-stdunit) ret-angle-list))
    )
    (reverse ret-angle-list)
  )
)

(defun send-command (robot ri cmd-msg)
  ;; precondition validation
  (assert-isinstance robot robot-model)
  (assert-isinstance ri robot-interface)
  (assert-isinstance cmd-msg mohou_ros::ControlCommand)

  ;; main
  (let* (
         (joint-name-list (send cmd-msg :joint_names))
         (angle-list-stdunit (vec-to-list (send cmd-msg :angles)))
         (angle-list-eusunit (convert-joint-angle-list robot joint-name-list angle-list-stdunit))
        )
    (assert-listp angle-list-eusunit)
    (dotimes (i (length joint-name-list))
      (let* (
             (angle-eusunit (nth i angle-list-eusunit))
             (joint-name (nth i joint-name-list))
             (joint-method-symbol (joint-name-to-symbol joint-name))
            )
        (assert-isinstance robot robot-model)
        (send robot joint-method-symbol :joint-angle angle-eusunit)))
    (send ri :angle-vector (send robot :angle-vector))
  )
)
