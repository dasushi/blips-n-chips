#!/bin/bash
echo --Clean--
rm -r benchmarksInstr
mkdir benchmarksInstr

echo --BubbleSort--
python convertBenchToMipsInstr.py -i benchmarks/BubbleSort.x -o benchmarksInstr/BubbleSortInstr.x

echo --CheckVowel--
python convertBenchToMipsInstr.py -i benchmarks/CheckVowel.x -o benchmarksInstr/CheckVowelInstr.x

echo --fact--
python convertBenchToMipsInstr.py -i benchmarks/fact.x -o benchmarksInstr/factInstr.x

echo --SimpleAdd--
python convertBenchToMipsInstr.py -i benchmarks/SimpleAdd.x -o benchmarksInstr/SimpleAddInstr.x

echo --SimpleIF--
python convertBenchToMipsInstr.py -i benchmarks/SimpleIf.x -o benchmarksInstr/SimpleIfInstr.x

echo --SumArray--
python convertBenchToMipsInstr.py -i benchmarks/SumArray.x -o benchmarksInstr/SumArrayInstr.x

echo --Swap--
python convertBenchToMipsInstr.py -i benchmarks/Swap.x -o benchmarksInstr/SwapInstr.x

