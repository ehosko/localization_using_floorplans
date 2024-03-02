# Localization using Floor Plans

This repository is an extension to the [Optimal Active Guidance Using Prior Floorplans](https://github.com/ehosko/optimal_active_guidance_in_mixed_reality_using_prior_floorplans/). It consists of two main components:

1. Floor plan Localization
2. Optimal Trajectory computation and coverage logging

When following the steps described in the [overview repository](https://github.com/ehosko/Active-Mapping-with-Known-Floor-Plans-for-Precise-Re-Localization), there are no additional installation steps required here.

## Floor Plan Localization
The floor plan localization is based on the following publication:

  ```bibtex
  @inproceedings{watanabe2020robust,
  title={Robust localization with architectural floor plans and depth camera},
  author={Watanabe, Yoshiaki and Amaro, Karinne Ramirez and Ilhan, Bahriye and Kinoshita, Taku and Bock, Thomas and Cheng, Gordon},
  booktitle={2020 IEEE/SICE International Symposium on System Integration (SII)},
  pages={133--138},
  year={2020},
  organization={IEEE}
}
  ```

The implementation aims to refine the SLAM pose predictions by matching the point cloud given by the depth image to a point cloud generated from the floor plan.

## Optimal Trajectory Computation
Given a floor plan, an optimal trajectory can be computed by transforming the floor plan into a graph and solving the travelling salesperson problem (TSP).

For more information, you can refer to the [overview repository](https://github.com/ehosko/Active-Mapping-with-Known-Floor-Plans-for-Precise-Re-Localization).




