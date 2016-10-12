;;;
;;; Copyright (c) 2016, Mihai Pomarlan <blandc@cs.uni-bremen.com>
;;; All rights reserved.
;;; 
;;; Redistribution and use in source and binary forms, with or without
;;; modification, are permitted provided that the following conditions are met:
;;; 
;;;     * Redistributions of source code must retain the above copyright
;;;       notice, this list of conditions and the following disclaimer.
;;;     * Redistributions in binary form must reproduce the above copyright
;;;       notice, this list of conditions and the following disclaimer in the
;;;       documentation and/or other materials provided with the distribution.
;;;     * Neither the name of Willow Garage, Inc. nor the names of its
;;;       contributors may be used to endorse or promote products derived from
;;;       this software without specific prior written permission.
;;; 
;;; THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
;;; AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
;;; IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
;;; ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
;;; LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
;;; CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
;;; SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
;;; INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
;;; CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
;;; ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
;;; POSSIBILITY OF SUCH DAMAGE.
;;;

(in-package :olivaw)

(defclass trajectory ()
  ())

(defclass tube-sample ()
  ((trajectory
     :initarg :trajectory
     :initform nil
     :reader sample-trajectory
     :type (or trajectory null))
   (score
     :initarg :score
     :initform 0
     :accessor sample-score
     :type number)
   (width
     :initarg :width
     :initform 1.0
     :accessor sample-width
     :type number)))

(defclass motion-tube ()
  ((samples
     :initarg :samples
     :initform nil
     :accessor tube-samples
     :type (or (list tube-sample) null))))

(defclass skill-type ()
  ((name
     :initarg :skill-name
     :initform nil
     :reader skill-name)
   (length-threshold
     :initarg :length-threshold
     :initform 0
     :reader length-threshold
     :type number)
   (curvature-threshold
     :initarg :curvature-threshold
     :initform 0
     :reader curvature-threshold
     :type number)))

(defclass motion-skill ()
  ((motion-tube
     :initarg :tube
     :initform nil
     :accessor skill-tube
     :type motion-tube)
   (skill-type
     :initarg :skill-type
     :initform nil
     :reader skill-type
     :type skill-type)
   (trajectory-length
     :initarg :trajectory-length
     :initform nil
     :accessor trajectory-length
     :type number)
   (curvature
     :initarg :curvature
     :initform nil
     :accessor curvature
     :type number)))

(defgeneric get-trajectory-length (trajectory))
(defmethod get-trajectory-length (trajectory trajectory)
  )
(defmethod get-trajectory-length ((tube tube-sample))
  (get-trajectory-length (sample-trajectory tube)))
(defmethod get-trajectory-length ((tube tube-sample))
  (get-trajectory-length (sample-trajectory tube)))
(defmethod get-trajectory-length ((tube motion-tube))
  (get-trajectory-length (car (tube-samples morion-tube))))

(defgeneric get-trajectory-curvature (trajectory))
(defmethod get-trajectory-curvature (trajectory trajectory)
  )
(defmethod get-trajectory-curvature ((tube tube-sample))
  (get-trajectory-curvature (sample-trajectory tube)))
(defmethod get-trajectory-curvature ((tube tube-sample))
  (get-trajectory-curvature (sample-trajectory tube)))
(defmethod get-trajectory-curvature ((tube motion-tube))
  (get-trajectory-curvature (car (tube-samples morion-tube))))

(defun trajectory-to-trajectory-distance (tA tB)
  )

(defun tube-from-lambda (generator-lambda score-threshold)
  (let* ((candidates (apply generator-lambda (list score-threshold)))
         (candidates (mapcar (lambda (candidate)
                               (let* ((trajectory (first candidate))
                                      (score (second candidate)))
                                 (make-instance 'tube-sample
                                                :trajectory trajectory
                                                :score score
                                                :width 0.01)))
                             candidates)))
    (make-instance 'motion-tube
                   :samples candidates)))

(defun ensure-transform (loc)
  (cond
    ((typep loc cl-transforms:transform)
      loc)
    ((typep loc cl-transforms:pose)
      (cl-transforms:make-transform (cl-transforms:origin loc)
                                    (cl-transforms:orientation loc)))))

(defun ensure-pose (loc)
  (cond
    ((typep loc cl-transforms:pose)
      loc)
    ((typep loc cl-transforms:transform)
      (cl-transforms:make-pose (cl-transforms:translation loc)
                               (cl-transforms:rotation loc)))))

(defun get-suggested-object-pose (object-loc object-to-feature-loc suggested-feature-loc)
  (let* ((object-loc (ensure-transform object-loc))
         (object-to-feature-loc (ensure-transform object-to-feature-loc))
         (suggested-feature-loc (ensure-transform suggested-feature-loc)))
    (ensure-pose (cl-transform:transform* suggested-feature-loc (cl-transforms:transform-inv object-to-feature-loc)))))

(defun skill-applicability (skill required-skill-type required-length required-curvature)
  (let* ((skill-type (skill-type skill))
         (length-threshold (length-threshold skill-type))
         (curvature-threshold (curvature-threshold skill-type))
         (length-threshold (if (< (length-threshold required-skill-type) length-threshold)
                             (length-threshold required-skill-type)
                             length-threshold))
         (curvature-threshold (if (< (curvature-threshold required-skill-type) curvature-threshold)
                             (curvature-threshold required-skill-type)
                             curvature-threshold))
         (length (trajectory-length skill))
         (curvature (curvature skill))
         (dl (- length required-length))
         (dc (- curvature required-curvature))
         (dl (/ dl length-threshold))
         (dc (/ dc curvature-threshold)))
    (if (equal (skill-name skill-type) (skill-name required-skill-type))
      (/ 1 (* (+ (* dl dl) 1) (+ (* dc dc) 1)))
      0)))

(defun select-skill-internal (scored-skills best-score best-skill)
  (if (equal scored-skills nil)
    best-skill
    (let* ((cr-scored-skill (car scored-skills))
           (next-scored-skills (cdr scored-skills))
           (cr-score (second cr-scored-skill))
           (cr-skill (first cr-scored-skill))
           (best-skill (if (< best-score cr-score)
                         cr-skill
                         best-skill))
           (best-score (if (< best-score cr-score)
                         cr-score
                         best-score)))
      (select-skill-internal next-scored-skills best-score best-skill))))

(defun select-skill (skills skill-type required-trajectory)
  (let* ((required-length (get-trajectory-length required-trajectory))
         (required-curvature (get-trajectory-curvature required-trajectory))
         (scored-skills (mapcar (lambda (skill)
                                  (list skill (skill-applicability skill skill-type required-length required-curvature)))
                                skills))
         (best-score 0)
         (best-skill nil))
    (select-skill-internal scored-skills best-score best-skill)))

(defun select-start ()
  )

