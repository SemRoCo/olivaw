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
   (skill-description-thresholds
     :initarg :description-thresholds
     :initform nil
     :reader skill-description-thresholds
     :type list)))

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
   (skill-descriptions
     :initarg :descriptions
     :initform nil
     :accessor skill-descriptions
     :type list)))

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

(defun get-trajectory-start-config (trajectory)
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

(defun get-thresholds (skill-type required-skill-type)
  (let* ((required-skill-thresholds (skill-description-thresholds required-skill-type))
         (act-skill-thresholds (skill-description-thresholds skill-type))
         (thresholds (mapcar (lambda (req-skill-threshold)
                               (let* ((req-name (car req-skill-threshold))
                                      (req-val (cadr req-skill-threshold))
                                      (act-val (cadr (assoc req-name act-skill-thresholds :test #'equal)))
                                      (val (if (or (not act-val) (< req-val act-val))
                                             req-val
                                             act-val)))
                                 (list req-name val)))
                             required-skill-thresholds)))
    thresholds))
(defun get-skill-score (values required-values thresholds)
  (let* ((scores (mapcar (lambda (req-val)
                           (let* ((req-value-name (car req-val))
                                  (req-value (cadr req-val))
                                  (act-value (cadr (assoc req-value-name values :test #'equal)))
                                  (threshold (cadr (assoc req-value-name thresholds :test #'equal))))
                             (if (and act-value threshold)
                               (let* ((dv (- act-value req-value))
                                      (dv (/ dv threshold)))
                                 (/ 1 (+ (* dv dv) 1)))
                               0)))
                         required-values)))
    (apply #'* scores)))
(defun skill-applicability (skill required-skill-type required-values)
  (let* ((skill-type (skill-type skill))
         (thresholds (get-thresholds skill-type required-skill-type))
         (values (skill-descriptions skill))
         (score (get-skill-score values required-values thresholds)))
    (if (equal (skill-name skill-type) (skill-name required-skill-type))
      score
      0)))

(defun select-max-scored-obj (scored-objects best-score best-object)
  (if (equal scored-objects nil)
    best-object
    (let* ((cr-scored-object (car scored-objects))
           (next-scored-objects (cdr scored-objects))
           (cr-score (second cr-scored-object))
           (cr-object (first cr-scored-object))
           (best-object (if (< best-score cr-score)
                          cr-object
                          best-object))
           (best-score (if (< best-score cr-score)
                         cr-score
                         best-score)))
      (select-max-scored-obj next-scored-objects best-score best-object))))

(defun select-skill (skills skill-type required-values)
  (let* ((scored-skills (mapcar (lambda (skill)
                                  (list skill (skill-applicability skill skill-type required-values)))
                                skills))
         (best-score 0)
         (best-skill nil))
    (select-max-scored-obj scored-skills best-score best-skill)))

(defun get-optimal-p (d w m)
  ;; Lots of magic here, but actually this is solving a cubic equation that corresponds to maximizing the
  ;; score function
  (if (< 0.01 d)
    (let* ((r (/ d w))
           (q (/ w m))
           (qq (* q q))
           (rr (* r r))
           (qqqq (* qq qq))
           (rrrr (* rr rr))
           (qqqqqq (* qq qqqq))
           (rrrrrr (* rr rrrr))
           (aux (+ (* qq (- 0 rr)) (* 2 qq) 2))
           (numr (- 0 (* aux 1.25992)))
           (dena (+ (* qqqq rrrr 432) (* qqqqqq rrrr -432)))
           (denb aux)
           (den (+ (* dena dena) (* denb denb denb qqqqqq rrrrrr 6912)))
           (den (sqrt den))
           (den (+ dena den))
           (den (exp (* (/ 1 3) (log den))))
           (ft (/ numr den))
           (st (* (/ 1 (* qq rr)) 0.0661417 den))
           (p (+ ft st 0.5)))
      (if (< 1 p)
        1
        (if (< p 0)
          0
          p)))
    0))

(defun interpolate-transform (start end p)
  (let* ((diff (cl-transforms:transform* (cl-transforms:transform-inv start) end)))
    (multiple-value-bind (axis angle) (cl-transforms:quaternion->axis-angle (cl-transforms:rotation diff))
      (when (< (abs angle) 0.001)
        (setf axis (cl-transforms:make-3d-vector 0 0 1)))
      (let* ((axis (cl-transforms:normalize-vector axis))
             (angle (* angle p))
             (diff-quat (cl-transforms:axis-angle->quaternion axis angle))
             (diff-tran (cl-transforms:v* (cl-transforms:translation diff)
                                          p))
             (diff (cl-transforms:make-transform diff-tran diff-quat)))
        (cl-transforms:transform* start diff)))))

(defun select-start (skills skill-type required-values object-loc object-to-feature-loc move-affinity transform-distance-fn)
  (let* ((skill (select-skill skills skill-type required-values))
         (samples (tube-samples (skill-tube skill)))
         (object-loc (ensure-transform object-loc))
         (object-to-feature-loc (ensure-transform object-to-feature-loc))
         (scored-starts (mapcar (lambda (sample)
                                  (let* ((sample-score (sample-score sample))
                                         (sample-width (sample-width sample))
                                         (sample-start (get-trajectory-start-config (sample-trajectory sample)))
                                         (candidate-start (get-suggested-object-pose object-loc object-to-feature-loc sample-start))
                                         (d (funcall transform-distance-fn candidate-start oject-loc))
                                         (p (get-optimal-p d sample-width move-affinity))
                                         (candidate-start (interpolate-transform object-loc candidate-start p))
                                         (dena (* (- 1 p) (/ d sample-width)))
                                         (denb (* p (/ d move-affinity)))
                                         (dena (* dena dena))
                                         (denb (* denb denb))
                                         (den (* (+ 1 dena) (+ 1 denb)))
                                         (score (* den sample-score)))
                                    (list candidate-start score)))
                                samples)))
    (select-max-scored-obj scored-starts 0 nil)))

