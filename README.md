pcd2bmp
===

Calculates 2D projection of a point cloud and saves it to a bitmap file.

Usage
---

	3DProjection: Projects point cloud data to a bitmap
	Usage: 3DProjection [-c x,y,z] [-r q1,q2,q3,q4] [-f fl] [-p psz] [-z] [-v] <src.pcd> <dest.bmp>
		-c (camera coordinates)
		-r (camera rotation quaternion)
		-f (camera focal length)
		-p (point size)
		-z (zoom out to fit points)
		-v (verbose logging)

Building:

	make all

C/C++ API
---

	#include "pcd2bmp.h"
	
	char* src = "src.pcd";
	char* dest = "dest.bmp";
	int pointSize = 1;

	pcd2bmp(src,dest,pointSize);

Example
---

	./pcd2bmp -c 0,0,-2 -p 5 -z example.pcd example.bmp

![](example.bmp)

	./pcd2bmp  -c 1,1,-2 -r 0.924,-0.383,0,0 -p 3 -z example.pcd example2.bmp
	
![](example2.bmp)
	
References
---

http://pointclouds.org/documentation/tutorials/pcd_file_format.php
