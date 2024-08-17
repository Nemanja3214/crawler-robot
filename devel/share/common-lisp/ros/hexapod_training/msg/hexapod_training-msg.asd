
(cl:in-package :asdf)

(defsystem "hexapod_training-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "ResultMsg" :depends-on ("_package_ResultMsg"))
    (:file "_package_ResultMsg" :depends-on ("_package"))
  ))