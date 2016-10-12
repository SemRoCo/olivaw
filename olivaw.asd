; -*- Mode: Lisp; Syntax: ANSI-Common-Lisp; Base: 10 -*-

(in-package :cl-user)

(asdf:defsystem olivaw
  :name "olivaw"
  :author "Mihai Pomarlan <blandc@cs.uni-bremen.com>"
  :maintainer "Mihai Pomarlan <blandc@cs.uni-bremen.com>"
  :licence "BSD"
  :description "Supportive maneuver planner for giskard."
  :depends-on (:cl-transforms-stamped)
  :components
  ((:module "src"
            :components
            ((:file "package")
             (:file "motion-tube" :depends-on ("package"))))))
