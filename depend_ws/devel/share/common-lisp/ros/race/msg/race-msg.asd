
(cl:in-package :asdf)

(defsystem "race-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "pid_input" :depends-on ("_package_pid_input"))
    (:file "_package_pid_input" :depends-on ("_package"))
  ))