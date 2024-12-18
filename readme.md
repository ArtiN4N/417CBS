# CMPT 417 Group SJG
Parallel Conflict Based Search with heuristics

repo: https://github.com/ArtiN4N/417CBS

# TO REPRODUCE
Run on CSIL or Ubuntu linux for consistency

## Visualizer
If you are on an actual machine, you can compile the visualizer via:
```
make vis_test_linux
```
you can run via:
```
bin/vis_test
```
The visualizer is set to load map 49. If you really care, you can go into the file and change which map file it loads.

Keep in mind the visualizer is for debugging purposes. Without particular dev libraries installed, it will not run correctly, as it relies on the raylib library.

## Data
All data is using maps from https://movingai.com/benchmarks/mapf/index.html


All compiled data can be found in the archive, in autotest/tests

Data is formatted like so:
> map{map #}scen1t{thread count}{modifier}.out

with modifier being "", "dg", "wdg" for cg, dg, and wdg results

If you would like to regenerate the data yourself:

### Test compilation
```
make cg_test
make dg_test
make wdg_test
```

### Test execution
using
> modifier = { cg, dg, wdg }

run:
```
bin/<modifier>_test <map #> <parallel? (0 or 1)> <threadcount>
```

This will generate the test files, formatted as described earlier, for the inputted map number, and threadcount. Use a 0 for parallel input if testing for serial, and 1 for testing parallel.

For example, to run tests for map 1, cg heuristic, serial version, run:
```
make cg_test
bin/cg_test 1 0 1
```


## Questions
If you have any issues with compilation, or understanding the generated data, please email me at jma246@sfu.ca