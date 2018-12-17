{-# LANGUAGE BangPatterns, OverloadedStrings #-}
module Graph (
      Graph
    , cost
    , generateGraph
    , numNodes
    , showInput
    ) where

import Control.Monad (when)
import Control.Monad (mzero, guard, forM, forM_)
import Control.Monad.Trans.Maybe (runMaybeT, MaybeT)
import Control.Monad.IO.Class (liftIO)
import Control.Monad.ST
import Control.Monad.Trans.Class (lift)
import Data.Array.IArray
import Data.Array.ST hiding (unsafeFreeze)
import Data.Array.Unboxed
import Data.Array.MArray as MArray hiding (unsafeFreeze)
import Data.Array.Unsafe (unsafeFreeze)
import qualified Data.ByteString.Lazy as B
import qualified Data.ByteString.Lazy.Char8 as B8
import Data.Monoid 

newtype Graph = Graph (UArray (Int, Int) Cost)      -- ^ an immutable graph
type Cost = Int
type Edge = ((Int, Int), Cost)
inf = (maxBound :: Int)

------------ Immutable functions -------------

-- | Return the number of nodes in graph @g@.
numNodes :: Graph -> Int
numNodes (Graph g) = n + 1
  where
    (_, (n, _)) = bounds g

-- | Return the element (cost) stored at position @i@, @j@ in graph @g@.
cost :: Graph -> Int -> Int -> Cost
cost (Graph g) i j = g ! (i, j)

-- | Process one input line, in the format of a list [src, tgt, cost].
-- If the input line is malformed, return Nothing. Otherwise, return the
-- corresponding edge.
edgeFromList :: [Int] -> Maybe Edge
edgeFromList [src, tgt, cost] = Just ((src, tgt), cost)
edgeFromList _ = Nothing


----------- Mutable functions --------------

-- | Process all input lines. If any input line is malformed, return Nothing.
-- Otherwise return the corresponding graph.
fromList :: Int -> [[Int]] -> MaybeT (ST s) (STUArray s (Int, Int) Cost)
fromList n input = do
    edges <- maybe mzero return $ sequence . map edgeFromList $ input
    m <- lift $ newListArray ((0, 0), (n - 1, n - 1)) $ repeat inf -- fill with inf values
    lift $ do
        mapM_ (\i -> writeArray m (i, i) 0) [0 .. n - 1]      -- zeroes main diagonal
        mapM_ (\((i, j), c) -> writeArray m (i - 1, j - 1) c >> writeArray m (j - 1, i - 1) c)
              edges
    return m

-- | Computes a graph containing the shortest path from every node. Assumes
-- all edges have a cost associated.
shortestPath :: STUArray s (Int, Int) Cost -> ST s ()
shortestPath m = do
    (_, (n, _)) <- getBounds m 
    let indices = [0 .. n] -- n is already adjusted by minus 1
    forM_ indices $ \i -> do
        forM_ indices $ \j -> do
            c <- readArray m (j, i)
            if c < inf
            then forM_ indices $ \k -> do
                c' <- readArray m (i, k)
                if c' < inf
                then do
                     let s   = c + c'
                     old <- readArray m (k, j)
                     if s < old
                     then do writeArray m (j, k) s
                             writeArray m (k, j) s
                     else return ()
                else return ()
            else return ()
    return ()

-- | Based on the input data, generate an immutable graph. Return Nothing
-- in case there was a parse error.
generateGraph :: Int -> [[Int]] -> Maybe Graph
generateGraph n input = case runST m of
    Just arr -> Just (Graph arr)
    _ -> Nothing
  where
    m :: ST s (Maybe (UArray (Int, Int) Cost))
    m = do mg <- runMaybeT $ fromList n input
           case mg of
               Nothing -> return Nothing
               Just mg' -> do
                   shortestPath mg'
                   arr <- unsafeFreeze mg'
                   return . Just $ arr
    
showInput :: Int -> Int -> Graph -> B.ByteString
showInput n p gr@(Graph g) =
    "data;\n"
    <> "param p := " <> B8.pack (show p) <> ";\n"
    <> "param n := " <> B8.pack (show n) <> ";\n"
    <> "param d : \t"
    <> B.concat (map (("\t" <>) . B8.pack . show) nodes) <> " :=\n"
    <> B.concat (map (\i -> ("\t\t\t" <> B8.pack (show i)
                       <> B.concat (map (\j -> "\t"
                            <> B8.pack (show (g ! (i, j)))) nodes)
                       <> "\n"))
               nodes)
    <> "\t;\nend;"
  where
    nodes = [0 .. n - 1]

