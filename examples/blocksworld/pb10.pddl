(define (problem pb10)
   (:domain blocksworld)
   (:objects a b c d e f g h i j )
   (:init (onTable a) (onTable b) (onTable c) (onTable d) (onTable e) 
          (onTable f) (onTable g) (onTable h) (onTable i) (onTable j) 
          (clear a)  (clear b) (clear c) (clear d) (clear e) (clear j) 
          (clear f)  (clear g) (clear h) (clear i)
          (equal a a) (equal b b) (equal c c) (equal d d) (equal e e)
          (equal f f) (equal g g) (equal h h) (equal i i) (equal j j)
          )
   (:goal (and (on a b) (on b c) (on c d) (on d e) (on e f) (on f g)
               (on g h) (on h i) (on i j) )))