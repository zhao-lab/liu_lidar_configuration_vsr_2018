# An Optimal LiDAR Configuration Approach

This repository contains the implementation of the algorithm in the paper [Where Should We Place LiDARs on the Autonomous Vehicle?-An Optimal Design Approach](https://arxiv.org/abs/1809.05845).
We use the perception area and non-detectable subspace to construct the design procedure as solving a min-max optimization problem and propose a bio-inspired measure -- volume to surface area ratio (VSR) -- as an easy-to-evaluate cost function representing the notion of the size of the non-detectable subspaces of a given configuration. We then adopt a cuboid-based approach to show that the proposed VSR-based measure is a well-suited proxy for object detection rate. We use the Artificial Bee Colony (ABC) evolutionary optimization algorithm to yield a tractable cost function computation. The implementation of the ABC algorithm is based on this repo: [https://github.com/rwuilbercq/Hive](https://github.com/rwuilbercq/Hive). Our experiments highlight the effectiveness of our proposed VSR measure in terms of cost-effectiveness configuration as well as providing insightful analyses that can improve the design of AV systems.

---------------------------------------------------------------

## How to run

Run the algorithm with the configuration in the `config.yml` file.

```
python main.py
```
Test the average running time of the solver with the current configuration problem.
```
python test.py
```
---------------------------------------------------------------

## Code organization

The `Evaluator` folder contains the C++ implementation of our solver. 
The `Evaluator.py` is the python interface of the solver.
The `Hive` folder is the ABC optimization algorithm implemented by [https://github.com/rwuilbercq/Hive](https://github.com/rwuilbercq/Hive). 
The `Result` folder saves the running results.

