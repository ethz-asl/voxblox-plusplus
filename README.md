# Voxblox++


<img src="https://github.com/ethz-asl/voxblox-plusplus/wiki/images/office_floor_map.png" width=700>

**Voxblox++** is a framework for incrementally building volumetric object-centric maps during online scanning with a localized RGB-D camera. Besides accurately describing the geometry of the reconstructed scene, the built maps contain information about the individual object instances observed in the scene. In particular, the proposed framework retrieves the dense shape and pose of recognized semantic objects, as well as of newly discovered, previously unobserved object-like instances.



### Build status

| Ubuntu 16.04 <br> + ROS kinetic | Ubuntu 18.04 <br> + ROS melodic|
|:---:|:---:|
|[![Build Status](https://jenkins.asl.ethz.ch/job/voxblox-plusplus-nightly/label=ubuntu-xenial/badge/icon)](https://jenkins.asl.ethz.ch/job/voxblox-plusplus-nightly/label=ubuntu-xenial/)|[![Build Status](https://jenkins.asl.ethz.ch/job/voxblox-plusplus-nightly/label=ubuntu-bionic/badge/icon)](https://jenkins.asl.ethz.ch/job/voxblox-plusplus-nightly/label=ubuntu-bionic/)|


# Getting started
- [Installing on Ubuntu](https://github.com/ethz-asl/voxblox_gsm/wiki/Installation-on-Ubuntu)
- [Basic usage](https://github.com/ethz-asl/voxblox-plusplus/wiki/Basic-usage)

More information can be found in the [wiki pages](https://github.com/ethz-asl/voxblox-plusplus/wiki).

# Citing
If you use the **Voxblox++** framework in your research, please cire the following publication:

- Margarita Grinvald, Fadri Furrer, Tonci Novkovic, Jen Jen Chung, Cesar Cadena, Roland Siegwart, Juan Nieto, **Volumetric Instance-Aware Semantic Mapping and 3D Object Discovery**, _IEEE Robotics and Automation Letters_, 2019. [[PDF](https://arxiv.org/abs/1903.00268)] [[Video](https://www.youtube.com/watch?v=Jvl42VJmYxg)]


```bibtex
@article{grinvald2019volumetric,
  title={{Volumetric Instance-Aware Semantic Mapping and 3D Object Discovery}},
  author={Grinvald, Margarita and Furrer, Fadri and Novkovic, Tonci and Chung, Jen Jen and Cadena, Cesar and Siegwart, Roland and Nieto, Juan},
  journal={IEEE Robotics and Automation Letters},
  year={2019},
  note={Under review}
}
```
