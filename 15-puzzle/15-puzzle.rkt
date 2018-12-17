#lang racket

;; ----------------------------------------------------------------------------
;; Imports

(module+ test
         (require rackunit))

;; ----------------------------------------------------------------------------
;; Datatypes

(struct posn (row col) #:transparent)

(struct state (matrix empty-slot) #:transparent)

;; ----------------------------------------------------------------------------
;; Constants

(define side-size 4)

(define initial-state
  (state
    (vector
      15  14  1  6
       9  11  4 12
       0  10  7  3
       13  8  5  2)
    (posn 2 0)))


(define goal-state
  (state
    #( 0  1  2  3
       4  5  6  7
       8  9 10 11
      12 13 14 15)
    (posn 0 0)))

;; ----------------------------------------------------------------------------
;; Functions

(define (matrix-ref matrix row col)
  (vector-ref matrix (+ (* row side-size) col)))

(define (matrix-set! matrix row col val)
  (vector-set! matrix
               (+ (* row side-size) col)
               val))

(define (target-state? st)
  (define matrix (state-matrix st))
  (and (equal? (posn 0 0) (state-empty-slot st))
       (for*/and ([row (in-range 0 side-size 1)]
                  [col (in-range 0 side-size 1)])
                 (let* ([element (matrix-ref matrix row col)])
                   (equal? element (+ (* row side-size)
                                      col))))))

(define (movement-valid? direction empty-slot)
  (match direction
         ['up (< (posn-row empty-slot) (- side-size 1))]
         ['down (> (posn-row empty-slot) 0)]
         ['left (< (posn-col empty-slot) (- side-size 1))]
         ['right (> (posn-col empty-slot) 0)]))

; assumes move direction is valid (see movement-valid?).
; Returns a new state
(define (move st direction)
  (define m (vector-copy (state-matrix st)))
  (define empty-slot (state-empty-slot st))
  (define r (posn-row empty-slot))
  (define c (posn-col empty-slot))
  (define new-empty-slot
    (match direction
           ['up  (begin (matrix-set! m r c (matrix-ref m (+ r 1) c))
                        (matrix-set! m (+ r 1) c 0)
                        (posn (+ r 1) c))]
           ['down (begin (matrix-set! m r c (matrix-ref m (- r 1) c))
                         (matrix-set! m (- r 1) c 0)
                         (posn (- r 1) c))]
           ['left (begin (matrix-set! m r c (matrix-ref m r (+ c 1)))
                         (matrix-set! m r (+ c 1) 0)
                         (posn r (+ c 1)))]
           ['right (begin (matrix-set! m r c (matrix-ref m r (- c 1)))
                          (matrix-set! m r (- c 1) 0)
                          (posn r (- c 1)))]))
  (state m new-empty-slot))

(define (l1-distance posn0 posn1)
  (+ (abs (- (posn-row posn0) (posn-row posn1))) 
     (abs (- (posn-col posn0) (posn-col posn1)))))

; computes the L1 distance from the current position and the goal position
(define (element-cost val current-posn)
  (define target-row (quotient val side-size))
  (define target-col (remainder val side-size))
  (l1-distance current-posn (posn target-row target-col)))

(define (state-cost st)
  (define m (state-matrix st))
  (for*/fold ([sum 0])
             ([i (in-range 0 side-size)]
              [j (in-range 0 side-size)])
     (+ sum (element-cost (matrix-ref m i j) (posn i j)))))

(define (next-states st)
  (define empty-slot (state-empty-slot st))
  (define valid-movements
    (filter (lambda (dir) (movement-valid? dir empty-slot))
          '(up down left right)))
  (map (lambda (dir) (move st dir)) valid-movements))

(define (display-state st)
  (define m (state-matrix st))
  (begin
    (for ([i (in-range 0 side-size 1)])
         (displayln "")
         (for ([j (in-range 0 side-size 1)])
              (printf "~a\t" (matrix-ref m i j))))
    (displayln "")))

;; ----------------------------------------------------------------------------
;; Tests

(module+ test
         (check-false (target-state? initial-state))
         (check-true (target-state? goal-state)))

(module+ test
         (check-true (movement-valid? 'up    (posn 0 0)))
         (check-true (movement-valid? 'left  (posn 0 0)))
         (check-false (movement-valid? 'right (posn 0 0)))
         (check-true (movement-valid? 'down  (posn 3 3))))

(module+ test
         (define source-state
           (state (vector
                    0 1 2 3
                    4 5 6 7
                    8 9 10 11
                    12 13 14 15)
                  (posn 0 0)))
         (define correct-target-state
           (state #(1 0 2 3
                      4 5 6 7
                      8 9 10 11
                      12 13 14 15)
                  (posn 0 1)))
         (define wrong-target-state
           (state #(4 1 2 3
                      0 5 6 7
                      8 9 10 11
                      12 13 14 15)
                  (posn 1 0)))
         (check-equal? (move source-state 'left) correct-target-state)
         (check-not-equal? (move source-state 'left) wrong-target-state))

(module+ test
         (check-eq? (l1-distance (posn 1 1) (posn 3 2)) 3)
         (check-eq? (l1-distance (posn 0 0) (posn 0 0)) 0)
         (check-eq? (l1-distance (posn 3 3) (posn 3 3)) 0)
         (check-eq? (l1-distance (posn 3 3) (posn 1 2)) 3))

(module+ test
         (check-eq? (element-cost 15 (posn 0 0)) 6)
         (check-eq? (element-cost 0 (posn 0 0)) 0)
         (check-eq? (element-cost 11 (posn 3 3)) 1)
         (check-eq? (element-cost 0 (posn 1 1)) 2))

(module+ test
         (check-eq? (state-cost goal-state) 0))
        
(module+ test
         (define nst
           (next-states initial-state))
         (check member
                (state
                  (vector
                    15  14  1  6
                    0  11  4 12
                    9  10  7  3
                    13  8  5  2)
                  (posn 1 0))
                nst)
         (check-eq? (length nst) 3))
