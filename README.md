# IKCharacterDeformer
Implementation of FK and IK using Eigen/ADOL-C solvers to let a character with joints and skinning weights be deformed in real-time, rendered in OpenGL. Project for CSCI 520.


- [IK Solver](https://github.com/meganfriedenberg/IKCharacterDeformer/blob/main/IKSolver/IK.cpp) performs IK by using ADOL-C to evaluate the gradient of FK (the Jacobian matrix) through Tikohonov regularization, and finally using Eigen to solve the systems of equations. Increase the alpha value to have more jagged movement, or decrease for smooth deformations.

- [FK Solver](https://github.com/meganfriedenberg/IKCharacterDeformer/blob/main/IKSolver/FK.cpp) performs FK that is needed for IK, updates local and global transforms of joints.

- [Linear Blend Skinning](https://github.com/meganfriedenberg/IKCharacterDeformer/blob/main/IKSolver/skinning.cpp) is applied using inputted skinning weights, change them out for differences in character. 


![GIF of armadillo character being deformed.](https://github.com/meganfriedenberg/meganfriedenberg.github.io/blob/master/images/IK.gif?raw=true)   
   

To better explain the [IK](https://github.com/meganfriedenberg/IKCharacterDeformer/blob/main/IKSolver/IK.cpp) math, take note of the following problem and equation:  
   
![A problem showing the optimization of Inverse Kinematics](https://github.com/meganfriedenberg/IKCharacterDeformer/blob/main/images/jacobian.JPG)   
  

Using ADOL-C the Jacobian (J) matrix is computed by taking the gradient of all first-order partial derivatives from FK. 

 ![An equation showing a simplified Tikohonov Regularization for how Inverse Kinematics is solved by Eigen.](https://github.com/meganfriedenberg/IKCharacterDeformer/blob/main/images/EigenSystem.JPG)  
   
Eigen is then used to solve a simplified equation of Tikohonov regularization once ADOL-C has computed the Jacobian, by solving for the change in angle.
   



