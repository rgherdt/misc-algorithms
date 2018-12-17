module Grasp (
      grasp
    , StopCriterium (..)
    , showSolution
    ) where

import Debug.Trace
import Data.Function (on)
import Data.Foldable (toList)
import Data.List
import qualified Data.IntMap as M
import System.CPUTime
import qualified Data.Vector as V
import qualified Graph as G
import System.Random

type Solution = V.Vector Int
type Cost = Int
inf = (maxBound :: Int)
data StopCriterium = TimeStop | IterStop

-- | Return a random solution to the problem: a p-size list of locations.
-- The solution is already sorted for efficiency reasons.
randomSolution :: StdGen -> Int -> Int -> Solution
randomSolution gen n p =
    V.fromList . sort . take p . nub . randomRs (0, n - 1) $ gen

-- | Return all 1-change neighbours from @sol@.
neighbours :: Int -> Solution -> [Solution]
neighbours n sol = do
    pos <- solIndices
    nbs <- map (\v -> sol V.// [(pos, v)]) candidates
    return nbs
  where
    candidates = [0 .. n - 1] \\ toList sol
    solIndices = [0 .. (V.length sol - 1)]

-- | Compute the value from solution @sol@.
solutionValue :: G.Graph -> Int -> Solution -> Cost
solutionValue g n sol =
    sum [minimum [G.cost g i j | j <- s] | i <- [0 .. n - 1]]
  where 
    s = toList sol

-- | Return a tuple containing the nearest and the second nearest facilities
-- to node @i@.
nearestFacilities :: G.Graph -> Solution -> Int -> (Int, Int)
nearestFacilities g sol i = case vs of
    [] -> error "empty nearest facilities"
    (a:b:_) -> (a, b)
  where
    vs = map fst . sortBy (compare `on` snd) . map (\j -> (j, G.cost g i j)) $
         toList sol

-- | Return a vector of all nearest and second nearest facilities to every 
-- node in the graph.
allNearestFacilities :: G.Graph -> Int -> Solution -> V.Vector (Int, Int)
allNearestFacilities g n sol =
    V.fromList $ map (nearestFacilities g sol) [0 .. n - 1]

-- | The Whitaker fast interchange algorithm. Return the best leaving
-- node from solution (together with it's profit) for a given incoming node @i@.
findOut :: G.Graph -> Solution -> V.Vector (Int, Int) -> Int -> (Cost, Int)
findOut g sol nearestVec i = (profit, leaving)
  where
    n = G.numNodes g
    nodes = V.fromList $ [0 .. n - 1] 
    (gaining, losing) =
        V.partition
            (\u -> G.cost g u i <= G.cost g u (fst $ nearestVec V.! u))
            $ V.fromList [0 .. n - 1]
    gain = V.sum $
            V.map (\u -> G.cost g u (fst $ nearestVec V.! u) - G.cost g u i)
                  gaining
    netloss u = (min (G.cost g u i) (G.cost g u (snd $ nearestVec V.! u)))
                - G.cost g u (fst $ nearestVec V.! u)
    netlossMap =
        foldr (\u acc -> M.adjust ((netloss u) +) (fst $ nearestVec V.! u) acc)
              (M.fromList . map (\v -> (v, 0)) $ toList sol)
              (toList losing)
    (leaving, loss) =
        head . sortBy (compare `on` snd) . M.toAscList $ netlossMap
    profit = gain - loss


-- | Optimized local search algorithm using findOut (Whitaker's fast interchange).
-- Return the local minimum, considering a 1-change neighbourhood from @sol@.
-- Uses a best improvement approach.
optLocalSearch :: G.Graph -> Solution -> Solution
optLocalSearch g s
    | profit best > 0 = optLocalSearch g (s V.// [(index, incoming)])
    | otherwise = s
  where
    Just index = V.findIndex (leaving ==) s
    n = G.numNodes g
    nearestVec = allNearestFacilities g n s -- is a vector of pairs
    best@(incoming, (bestProfit, leaving)) =
        maximumBy (compare `on` profit)
                 . map (\i -> (i, findOut g s nearestVec i)) $
                       [0 .. n - 1] \\ toList s
    profit = fst . snd 
    
-- | First improvement local search. Naive implementation.
localSearch :: G.Graph -> (Cost, Solution) -> (Cost, Solution)
localSearch g (v0, s0) = case betters of
    [] -> (v0, s0)
    _  -> localSearch g (head betters)
  where
    betters = filter (\(v', _) -> v' < v0) $ map compute $ neighbours n s0
    n = G.numNodes g
    compute s = (solutionValue g n s, s)

-- | Return a solution, choosen randomly from the alpha % best candidates
-- (those less distant to other nodes in the graph).
randomizedGreedy :: StdGen -> G.Graph -> Int -> Int -> Float -> (Solution, StdGen)
randomizedGreedy gen g n p alpha =
    (V.fromList s, gen')
  where
    (s, _, gen') =
        foldr (\_ (sol, remaining, gen) ->
                  let size = alphaSize (length remaining)
                      (v', gen') = pick gen $ rcl size remaining
                  in (v' : sol, delete v' remaining, gen'))
              ([], vs, gen)
              [0 .. p - 1]
    vs = [0 .. n - 1]
    totalCost i = sum $ map (G.cost g i) vs
    candidates vs = sortBy (compare `on` snd) $ map (\v -> (v, totalCost v)) vs
    alphaSize n' = round (alpha * fromIntegral n') :: Int
    rcl size remaining = map fst . take size $ candidates remaining
    pick :: StdGen -> [a] -> (a, StdGen)
    pick gen ls = (ls !! ix, gen')
      where
        (ix, gen') = randomR (0, length ls - 1) gen


-- | The grasp algorithm, applying the randomizedGreedy and local search 
-- algorithms until the given stop criterium is reached. Return a solution
-- and it's cost. The intermediate results are printed to standard output
-- during it's runtime.
grasp :: StdGen
      -> G.Graph
      -> Int
      -> Int
      -> Float
      -> Int
      -> StopCriterium
      -> Integer
      -> IO (Cost, Solution)
grasp gen g n p alpha counter0 stop startTime = do
    putStrLn $ "First solution value: " ++ show val0
    putStrLn $ "\nsolution\trunning time"
    go gen' counter0 val0 s0
  where
    (s0, gen') = randomizedGreedy gen g n p alpha
    val0 = solutionValue g n s0
    maxTime = fromIntegral counter0
    go gen counter val s
        | counter <= 0 = return (val, s)
        | val'' < val = do
            curTime <- getCPUTime
            let diffTime = fromIntegral (curTime - startTime) / 1000000000000
            putStrLn $ show val'' ++ "\t\t(" ++ show diffTime ++ " s)"
            case stop of
                IterStop -> go gen' (counter - 1) val'' s''
                TimeStop | diffTime >= maxTime -> return (val, s)
                         | otherwise -> go gen' counter val'' s'' -- doesn't decrement
        | otherwise = case stop of
                IterStop -> go gen' (counter - 1) val s
                TimeStop -> do
                    curTime <- getCPUTime
                    let diffTime = fromIntegral (curTime - startTime) / 1000000000000
                    if diffTime >= maxTime 
                        then return (val, s)
                        else go gen' counter val s -- doesn't decrement
      where
        (s', gen') = randomizedGreedy gen g n p alpha
        s'' = optLocalSearch g s'
        val'' = solutionValue g n s''
    
showSolution :: Solution -> Cost -> String
showSolution s val =
    "Solution: " ++ show (map (1 +) $ toList s) ++ "\nValue: " ++ show val
