#lang racket

(require racket/set)
(require data/heap)

;; ----------------------------------------------------------------------------
;; Datatypes

(struct posn (row col) #:transparent)

(struct state (matrix empty-slot) #:transparent)

(define directions '(up down left right))

(struct node (state prev g-value f-value) #:transparent)

;; ----------------------------------------------------------------------------
;; Constants

(define side-size 4)

(define initial-state
  (state
    #(
      15  14  1  6
      9  11  4 12
      0  10  7  3
      13  8  5  2)
    (posn 2 0)))

(define sample-state
  (state
    #( 1  8  2 12 
      5  6 0  4
      9  3  7 15
      13 10 14 11)
    (posn 1 2)))

(define sample-state2
  (state
    #( 5  8  7 11 
      1  6 12  2
      9  0  13 10
      14 3  4 15)
    (posn 2 1)))


(define almost-right-state
  (state
    #( 1  2  3  4
      5  6  7 8
      9  0 10 12
      13 14 11 15)
    (posn 2 1)))

(define initial-node
  (node initial-state
        #f
        0
        0))

(define goal-state
  (state
    #( 1  2  3  4
      5  6  7 8
      9 10 11 12
      13 14 15 0)
    (posn 3 3)))

;; ----------------------------------------------------------------------------
;; Functions

(define (matrix-ref matrix row col)
  (vector-ref matrix (+ (* row side-size) col)))

(define (matrix-set! matrix row col val)
  (vector-set! matrix
               (+ (* row side-size) col)
               val))

(define (target-state? st)
  (equal? st goal-state))


#|
(define (target-state? st)
  (define matrix (state-matrix st))
  (and (equal? (posn 3 3) (state-empty-slot st))
       (for*/and ([row (in-range 0 side-size 1)]
                  [col (in-range 0 side-size 1)])
                 (let* ([element (matrix-ref matrix row col)])
                   (equal? element (remainder (+ (* row side-size)
                                                 col
                                                 1)
                                              16))))))
|#

(define (reconstruct-movements leaf-node)
  (define (posn-diff p0 p1)
    (posn (- (posn-row p1) (posn-row p0))
          (- (posn-col p1) (posn-col p0))))

  (define (find-out-movement prev-st st)
    (let ([prev-empty-slot (state-empty-slot prev-st)]
          [this-empty-slot (state-empty-slot st)])
      (match (posn-diff prev-empty-slot this-empty-slot)
             [(posn  1  0) 'u]
             [(posn -1  0) 'd]
             [(posn  0  1) 'l]
             [(posn  0 -1) 'r]
             [#f 'invalid])))

  (define (iter n path)
    (if (or (not n) (not (node-prev n)))
        path
      (iter (node-prev n)
            (cons (find-out-movement (node-state n)
                                     (node-state (node-prev n)))
                  path))))
  (iter leaf-node '()))

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
  (if (= val 0)
      (l1-distance current-posn (posn 3 3))
    (let ([target-row (quotient (- val 1) side-size)]
          [target-col (remainder (- val 1) side-size)])
      (l1-distance current-posn (posn target-row target-col)))))

(define (state-l1-distance-to-goal st)
  (define m (state-matrix st))
  (for*/fold
    ([sum 0])
    ([i (in-range side-size)]
     [j (in-range side-size)])
    (let ([val (matrix-ref m i j)])
      (if (not (= val 0))
        (+ sum (element-cost (matrix-ref m i j) (posn i j)))
        sum))))

(define (state-cost st)
  (+ (state-l1-distance-to-goal st)
     (linear-conflicts st goal-state)))

;(define (linear-conflicts st0 st1)
;  (+ (all-row-conflicts st0 st1) (all-col-conflicts st0 st1)))


(define (out-of-order-values lst)
  (define (iter val-lst sum)
    (if (empty? val-lst)
      sum
      (let* ([val (car val-lst)]
             [rst (cdr val-lst)]
             [following-smaller-values
               (filter (lambda (val2) (> val2 val))
                       rst)])
           (iter rst (+ sum (length following-smaller-values))))))
  (* 2 (iter lst 0)))

(define (row-conflicts row st0 st1)
  (define m0 (state-matrix st0))
  (define m1 (state-matrix st1))
  
  (define values-in-correct-row
    (for/fold
      ([lst '()])
      ([col0 (in-range side-size)])
       (let* ([val0 (matrix-ref m0 row col0)]
              [in-goal-row?
               (for/first ([col1 (in-range side-size)]
                                  #:when (= val0 (matrix-ref m1 row col1)))
                                    #t)])
          (if in-goal-row? (cons val0 lst) lst))))
  (min 6 (out-of-order-values
             ; 0 doesn't lead to a linear conflict
             (filter positive? values-in-correct-row))))


(define (col-conflicts col st0 st1)
  (define m0 (state-matrix st0))
  (define m1 (state-matrix st1))
  (define values-in-correct-col
    (for/fold
      ([lst '()])
      ([row0 (in-range side-size)])
       (let* ([val0 (matrix-ref m0 row0 col)]
              [in-goal-col?
               (for/first ([row1 (in-range side-size)]
                                  #:when (= val0 (matrix-ref m1 row1 col)))
                                    #t)])
          (if in-goal-col? (cons val0 lst) lst))))
  (min 6 (out-of-order-values
           ; 0 doesn't lead to a linear conflict
             (filter positive? values-in-correct-col))))

(define (all-row-conflicts st0 st1)
  (for/fold ([sum 0])
            ([row (in-range side-size)])
            (+ (row-conflicts row st0 st1) sum)))

(define (all-col-conflicts st0 st1)
  (for/fold ([sum 0])
            ([col (in-range side-size)])
            (+ (col-conflicts col st0 st1) sum)))


(define (linear-conflicts st0 st1)
  (+ (all-row-conflicts st0 st1) (all-col-conflicts st0 st1)))

(define (next-state-dir-pairs current-node)
  (define st (node-state current-node))
  (define empty-slot (state-empty-slot st))
  (define valid-movements
    (filter (lambda (dir) (movement-valid? dir empty-slot))
            directions))
  (map (lambda (dir)
         (cons (move st dir) dir))
       valid-movements))

(define (display-state st)
  (define m (state-matrix st))
  (begin
    (for ([i (in-range 0 side-size 1)])
         (displayln "")
         (for ([j (in-range 0 side-size 1)])
              (printf "~a\t" (matrix-ref m i j))))
    (displayln "")))

(define (compare-nodes n0 n1)
  (<= (node-f-value n0) (node-f-value n1)))

(define (A* initial-st)
  (define open-lst (make-heap compare-nodes))
  (define initial-st-cost (state-cost initial-st))
  (printf "Initial cost: ~a\n" initial-st-cost)
  (heap-add! open-lst (node initial-st #f 0 (state-cost initial-st)))
  (define closed-set (mutable-set))

  (define dp-hash (make-hash))

  (hash-set! dp-hash initial-st 0)
  
  (define (pick-next-node!)
    (define next-node (heap-min open-lst))
    (heap-remove-min! open-lst)
    next-node)

  (define (sort-lst lst)
    (sort lst
          (lambda (n0 n1)
              (< (node-f-value n0) (node-f-value n1)))))

  (define (expand-node n)
    
    (define this-best-cost (hash-ref dp-hash (node-state n)))
    (define n-cost (node-g-value n))
    
    (define (iter lst)
      (if (empty? lst)
          '()
        (let* ([succ (car lst)]
               [succ-st (car succ)]
               [succ-dir (cdr succ)]
               [succ-st-best-cost (hash-ref dp-hash succ-st #f)]
               [succ-cost (+ 1 n-cost)])
          (if (set-member? closed-set succ-st)
              (iter (cdr lst))
            (if (and succ-st-best-cost
                     (> succ-cost succ-st-best-cost))
                (iter (cdr lst))
              (begin (hash-set! dp-hash succ-st succ-cost)
                     (heap-add! open-lst
                                (node succ-st
                                      n
                                      succ-cost
                                      (+ (state-cost succ-st)
                                         succ-cost)))
                     (iter (cdr lst))))))))
    (if (and this-best-cost
             (< this-best-cost n-cost))
        #f
      (let ([successors (next-state-dir-pairs n)])
        (iter successors)))) 
  
  (define counter 0)
  (define (loop)
    (define current-node (pick-next-node!))
    (define current-state (node-state current-node))
     (set! counter (+ counter 1))
          (if (= (remainder counter 10000) 0)
            (printf "~a ~a ~a\n" counter
		    (heap-count open-lst)
		    (node-g-value current-node))
            (void))
 
    (cond [(target-state? current-state)
           (let ([path (reconstruct-movements current-node)])
             (cons path (length path)))]
          [else
            (begin (set-add! closed-set (node-state current-node))
                   (expand-node current-node)
                   (if (= (heap-count open-lst) 0)
                       #f
                     (loop)))]))
  (loop))


;; ----------------------------------------------------------------------------
;; Tests

(module+
  test
  (require rackunit
           rackunit/text-ui)
  (define tests
    (test-suite
      "15-puzzle problem tests"
      
      (test-case
        "Predicate function tests."
        (check-false (target-state? initial-state))
        (check-true (target-state? goal-state))
        
        (check-true (movement-valid? 'up    (posn 0 0)))
        (check-true (movement-valid? 'left  (posn 0 0)))
        (check-false (movement-valid? 'right (posn 0 0)))
        (check-true (movement-valid? 'down  (posn 3 3))))
      
      
      (test-case
        "move tests"
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
      
      (test-case
        "cost tests"
        (check-eq? (l1-distance (posn 1 1) (posn 3 2)) 3)
        (check-eq? (l1-distance (posn 0 0) (posn 0 0)) 0)
        (check-eq? (l1-distance (posn 3 3) (posn 3 3)) 0)
        (check-eq? (l1-distance (posn 3 3) (posn 1 2)) 3)
        
        
        (check-eq? (element-cost 15 (posn 0 0)) 5)
        (check-eq? (element-cost 0 (posn 0 0)) 6)
        (check-eq? (element-cost 11 (posn 3 3)) 2)
        (check-eq? (element-cost 0 (posn 1 1)) 4)
        
        (check-eq? (state-cost goal-state) 0)
        (check-eq? (state-cost initial-state) 38)
        (check-eq? (state-cost almost-right-state) 3)
        )

      (test-case
        "linear conflicts test"
        (define other-state
         (state
           #( 1  2  6  4
             8  7  3 5
             9 11 10 0
             12 13 14 15)
           (posn 2 3)))
        (check-eq? (out-of-order-values '(3 2 1)) 0)
        (check-eq? (out-of-order-values '(3 1 2)) 2)
        (check-eq? (out-of-order-values '(1 3 2)) 4)
        (check-eq? (out-of-order-values '(1 2 3 4)) 12)
        (check-eq? (row-conflicts 0 other-state goal-state) 0) 
        (check-eq? (row-conflicts 1 other-state goal-state) 6) 
        (check-eq? (row-conflicts 2 other-state goal-state) 2) 
        (check-eq? (row-conflicts 3 other-state goal-state) 0) 
        (check-eq? (all-row-conflicts other-state goal-state) 8) 

        (check-eq? (col-conflicts 2 other-state goal-state) 0) 
        (check-eq? (col-conflicts 3 other-state goal-state) 0) 
        (check-eq? (all-col-conflicts other-state goal-state) 0) 
        (check-eq? (linear-conflicts other-state goal-state) 8)
        )
      
      (test-case
        "next-nodes test"
        (define ns
          (next-state-dir-pairs initial-node))
        (check member
                 (cons (state
                         (vector
                           15  14  1  6
                           0  11  4 12
                           9  10  7  3
                           13  8  5  2)
                         (posn 1 0))
                       'down)
               ns)
        (check-eq? (length ns) 3))
      (test-case
        "A* test"
        (define sample-state
          (state
            #( 1  2  3  4
              5  6  7 8
              9 10 11 12
              13 14 0 15)
            (posn 3 2)))
        (check-equal?
          (A* sample-state)
          (cons (list 'r) 1)))))
  (run-tests tests))

;(require profile)
;(module+ main
;  (profile-thunk (thunk (A* initial-state))))

(module+ main
  (A* initial-state))
