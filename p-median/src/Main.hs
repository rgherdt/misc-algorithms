{-# LANGUAGE OverloadedStrings #-}
module Main where

import Options.Applicative
import Control.Monad (liftM)
import qualified Graph as G
import Grasp (grasp, showSolution, StopCriterium (..))
import System.IO
import System.Random
import System.CPUTime
import qualified Data.ByteString.Lazy as B
import qualified Data.ByteString.Lazy.Char8 as B8

data Options = Options
    { optOutfile :: String
    , optNum :: Int
    , optIter :: Bool
    , optAlpha :: Float
    , optSeed :: Int
    }


parseOptions :: Parser Options
parseOptions = Options
    <$> argument str ( metavar "OUTFILE" )
    <*> option auto ( short 'n'
                   <> value 500
                   <> metavar "NUM"
                   <> help "Limit for chosen stop criterium (default: 500)."
                    )
    <*> switch ( short 'i'
              <> long "iter"
              <> help "If true, uses number of iterations (set by -n) as stop criterium"
               )
    <*> option auto ( long "alpha"
                   <> short 'a'
                   <> value 0.2
                   <> metavar "ALPHA"
                   <> help "Alpha parameter in (0.0, 1.0] (default: 0.2)"
                    )
    <*> option auto ( long "seed"
                   <> short 's'
                   <> value (- 1) 
                   <> metavar "SEED"
                   <> help "Semente para o gerador de números aleatórios"
                   )

opts = info (helper <*> parseOptions)
            ( fullDesc
           <> progDesc "Solve the p-median uncapacitaded problem with GRASP"
           <> header "A GRASP-based solver for the p-median uncapacitaded problem" )
            
main = do
    op <- execParser opts
    let iterNum = optNum op
        alpha = optAlpha op
        stop | optIter op = IterStop
             | otherwise = TimeStop
        outFile = optOutfile op
        seed = optSeed op 
    params <- liftM (map read . words) getLine :: IO [Int]
    matrix <- case params of
        [n, numEdges, p] -> do
            contents <- return . B8.lines =<< B8.getContents
            gen <- if seed == -1 then getStdGen else return $ mkStdGen seed
            let g = G.generateGraph n $
                     map (map read . map B8.unpack . B8.words) contents
            case g of
                Just g -> do
                    startTime <- getCPUTime
                    (val, s) <- grasp gen g n p alpha iterNum stop startTime
                    let resultStr = showSolution s val
                    putStrLn resultStr
                    writeFile outFile $ resultStr ++ "\n"
                _ -> B8.putStrLn "p-median: Inconsistent input graph"
            return ()
        _ -> fail "p-median: First input line wrong formatted."
    return ()

