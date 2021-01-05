# RRT (Rapidly Exploring Random Trees)
This repository contains implementation of RRT (Rapidly exploring Random Trees) path planning algorithm in Python and C++. The code has been documented for better readability and understanding.
The start, goal and obstacle co-ordinates can be changed from within the code itself.

## Python
The ```RRT Python``` folder contains the python file of the algorithm. 

#### Dependencies
<ul>
    <li>
        <a href="https://numpy.org/" >NumPy</a>
    </li>
    <li>
        <a href="https://matplotlib.org/" >Matplotlib</a>
    </li>
</ul>

#### Running code via command line
1. Change directory to where the code file is 
2. Execute command - ```python rrt_python.py```

## C++
The ```RRT C++``` folder contains the C++ files of the algorithm. It also contains the Eigen and Matplotlibcpp library files required for the project

#### Dependencis
<ul>
    <li>
        <a href="http://eigen.tuxfamily.org/index.php?title=Main_Page#License" >Eigen</a>
    </li>
    <li>
        <a href="https://github.com/lava/matplotlib-cpp" >Matplotlib-cpp</a>
    </li>
</ul>

#### Running code via Termial (Ubuntu)
1. Change the directory to where the code files are
2. Execute command - ```g++ main.cpp -I/path/to/Python.h/file -lpython-version```
In my case, the above is - ```g++ main.cpp -I/usr/include/python3.8 -lpython3.8```
3. After successfully executing above command. Execute - ```./a.out```


Checkout the RRT research paper <a href = "http://msl.cs.uiuc.edu/~lavalle/papers/LavKuf01.pdf">here</a>
