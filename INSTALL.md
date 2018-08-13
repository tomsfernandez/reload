# Instrucciones de instalación
## Dependencias 

Las dependencias necesarias para compilar el sistema son:
- OpenCV 3.2.0
- Eigen 3.3.4
- Boost Serialization 1.66.0
- Glew
- Pangolin
- g2o (Modificado por el autor original del proyecto, incluido en el mismo)
- DBoW2 (Modificado por el autor original del proyecto, incluido en el mismo)

Para testing se utilizan:
- gtest 1.8.0

Es recomendable crear una carpeta "libs" en el home para contener las dependencias que hay que instalar. 

## Instalación de OpenCV
Desde libs
```Shell
$ sudo apt-get install build-essential
$ sudo apt-get install cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
$ sudo apt-get install python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libdc1394-22-dev
$ sudo apt-get install wget
$ cd ~
$ mkdir libs
$ cd libs
$ wget https://github.com/opencv/opencv/archive/3.2.0.tar.gz
$ tar -xvzf 3.2.0.tar.gz
$ rm 3.2.0.tar.gz
$ cd opencv-3.2.0
$ mkdir build
$ cd build
$ sudo cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/usr/local .. (si no corre, correr devuelta)
$ sudo make install
```

## Instalación de Glew
```Shell
$ sudo apt-get install libglew-dev
```

## Instalación de Pangolin
Desde libs
```Shell
$ sudo git clone https://github.com/stevenlovegrove/Pangolin.git
$ cd Pangolin
$ mkdir build
$ cd build
$ cmake ..
$ cmake --build .
```

## Instalación del resto de las dependencias mediante Conan
Desde el root del proyecto
```Shell
$ sudo apt install python-pip
$ sudo pip install conan
$ mkdir build
$ cd build
$ conan remote add bincrafters https://api.bintray.com/conan/bincrafters/public-conan
$ conan install ..
```

## Instalación de DBoW2
Desde el root del proyecto
```Shell
$ cd deps
$ cd DBoW2
$ mkdir build
$ cd build
$ cmake .. -DCMAKE_BUILD_TYPE=Release
$ make -j3
```

## Instalación de g2o
Desde el root del proyecto
```Shell
$ cd deps
$ cd g2o
$ mkdir build
$ cd build
$ conan install ..
$ cmake .. 
$ make -j2
```
## Compilación del proyecto
Desde el root del proyecto
```Shell
$ cd build
$ cmake ..
$ make -j2
```
