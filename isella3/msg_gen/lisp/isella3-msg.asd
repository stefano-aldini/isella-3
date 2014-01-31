
(cl:in-package :asdf)

(defsystem "isella3-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "MyStateMessage" :depends-on ("_package_MyStateMessage"))
    (:file "_package_MyStateMessage" :depends-on ("_package"))
  ))