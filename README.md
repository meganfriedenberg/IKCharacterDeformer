# IKCharacterDeformer
Implementation of FK and IK using Eigen/ADOL-C solvers to let a character with joints and skinning weights be deformed in real-time, rendered in OpenGL. Project for CSCI 520.


- [IK Solver](https://github.com/meganfriedenberg/IKCharacterDeformer/blob/main/IKSolver/IK.cpp) performs IK by using ADOL-C to evaluate the gradient of FK (the Jacobian matrix) through Tikohonov regularization, and finally using Eigen to solve the systems of equations. Increase the alpha value to have more jagged movement, or decrease for smooth deformations.

- [FK Solver](https://github.com/meganfriedenberg/IKCharacterDeformer/blob/main/IKSolver/FK.cpp) performs FK that is needed for IK, updates local and global transforms of joints.

- [Linear Blend Skinning](https://github.com/meganfriedenberg/IKCharacterDeformer/blob/main/IKSolver/skinning.cpp) is applied using inputted skinning weights, change them out for differences in character. 


![GIF of armadillo character being deformed.](https://github.com/meganfriedenberg/meganfriedenberg.github.io/blob/master/images/IK.gif?raw=true)