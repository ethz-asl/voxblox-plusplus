# Voxblox++

[![Build Status](https://jenkins.asl.ethz.ch/buildStatus/icon?subject=ubuntu%2016.04%20%2B%20ROS%20kinetic&job=voxblox-plusplus-nightly%2Flabel%3Dubuntu-xenial)](https://jenkins.asl.ethz.ch/job/voxblox-plusplus-nightly/label=ubuntu-xenial/)
[![Build Status](https://jenkins.asl.ethz.ch/buildStatus/icon?subject=ubuntu%2018.04%20%2B%20ROS%20melodic&job=voxblox-plusplus-nightly%2Flabel%3Dubuntu-bionic)](https://jenkins.asl.ethz.ch/job/voxblox-plusplus-nightly/label=ubuntu-bionic/)

**Voxblox++** is a framework for incrementally building volumetric object-centric maps during online scanning with a localized RGB-D camera. Besides accurately describing the geometry of the reconstructed scene, the built maps contain information about the individual object instances observed in the scene. In particular, the proposed framework retrieves the dense shape and pose of recognized semantic objects, as well as of newly discovered, previously unobserved object-like instances.

<p align="center">
  <img src="https://github.com/ethz-asl/voxblox-plusplus/wiki/images/office_floor_map.png" width=700>
</p>


## Getting started
- [Installing on Ubuntu](https://github.com/ethz-asl/voxblox_gsm/wiki/Installation-on-Ubuntu)
- [Sample Datasets](https://github.com/ethz-asl/voxblox-plusplus/wiki/Sample-Datasets)
- [Basic usage](https://github.com/ethz-asl/voxblox-plusplus/wiki/Basic-usage)
- [The Voxblox++ node](https://github.com/ethz-asl/voxblox-plusplus/wiki/The-voxblox-plusplus-node)

More information and sample datasets can be found in the [wiki pages](https://github.com/ethz-asl/voxblox-plusplus/wiki).

## Citing
The **Voxblox++** framework is described in the following publication:

- Margarita Grinvald, Fadri Furrer, Tonci Novkovic, Jen Jen Chung, Cesar Cadena, Roland Siegwart, and Juan Nieto, **Volumetric Instance-Aware Semantic Mapping and 3D Object Discovery**, in _IEEE Robotics and Automation Letters_, July 2019. [[PDF](https://arxiv.org/abs/1903.00268)] [[Video](https://www.youtube.com/watch?v=Jvl42VJmYxg)]


```bibtex
@article{grinvald2019volumetric,
  author={M. {Grinvald} and F. {Furrer} and T. {Novkovic} and J. J. {Chung} and C. {Cadena} and R. {Siegwart} and J. {Nieto}},
  journal={IEEE Robotics and Automation Letters},
  title={{Volumetric Instance-Aware Semantic Mapping and 3D Object Discovery}},
  year={2019},
  volume={4},
  number={3},
  pages={3037-3044},
  doi={10.1109/LRA.2019.2923960},
  ISSN={2377-3766},
  month={July},
}
```

The original geometry-only framework was introduced in the following publication:

- Fadri Furrer, Tonci Novkovic, Marius Fehr, Abel Gawel, Margarita Grinvald, Torsten Sattler, Roland Siegwart, and Juan Nieto, **Incremental Object Database: Building 3D Models from Multiple Partial Observations**, in _IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)_, October 2018. [[PDF](https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=8594391)] [[Video](https://www.youtube.com/watch?v=9_xg92qqw70)]

```bibtex
@inproceedings{8594391,
  author={F. {Furrer} and T. {Novkovic} and M. {Fehr} and A. {Gawel} and M. {Grinvald} and T. {Sattler} and R. {Siegwart} and J. {Nieto}},
  booktitle={2018 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  title={{Incremental Object Database: Building 3D Models from Multiple Partial Observations}},
  year={2018},
  pages={6835-6842},
  doi={10.1109/IROS.2018.8594391},
  ISSN={2153-0866},
  month={Oct},
}
```

If you use **Voxblox++** in your research, please cite accordingly.

## License
The code is available under the [BSD-3-Clause license](https://github.com/ethz-asl/voxblox-plusplus/blob/master/LICENSE).
