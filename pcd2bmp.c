#include "pcd2bmp.h"
#define MARGIN 0.1f
#define DEFAULT_FOCAL_LENGTH 1.0f


int verboseLogging = 0;

void int2byte(int i, int numBytes, FILE* f) {
	int j;
	unsigned char *c = (unsigned char*)&i;
	for (j = 0; j < numBytes; j++) {
		fputc(c[j], f);
	}
}


Quaternion quaternionMult(Quaternion qa, Quaternion qb) {
	Quaternion qc;
	qc.r = qa.r*qb.r - qa.i*qb.i - qa.j*qb.j - qa.k*qb.k;
	qc.i = qa.r*qb.i + qa.i*qb.r + qa.j*qb.k - qa.k*qb.j;
	qc.j = qa.r*qb.j + qa.j*qb.r + qa.k*qb.i - qa.i*qb.k;
	qc.k = qa.r*qb.k + qa.k*qb.r + qa.i*qb.j - qa.j*qb.i;
	return qc;
}

Quaternion quaternionInv(Quaternion q) {
	Quaternion qc;
	qc.r = q.r;
	qc.i = -q.i;
	qc.j = -q.j;
	qc.k = -q.k;
	return qc;
}

Quaternion quaternionRot(Quaternion q, Quaternion vec) {
	return quaternionMult(quaternionMult(q, vec), quaternionInv(q));
}

//unused: implement later
PCD* NewPCDFromFile(char* fileName) {
	FILE* f = fopen(fileName, "r");
	if (!f) {
		printf("File not found: %s\n", fileName);
		return NULL;
	}
	char buf[256];
	PCD_file_format format_order[] = {
		VERSION,
		FIELDS,
		SIZE,
		TYPE,
		COUNT,
		WIDTH,
		HEIGHT,
		VIEWPOINT,
		POINTS,
		DATA,
		POINT_CLOUD };
	PCD_file_format* fm = &format_order[0];
	PCD* pcd = (PCD*)malloc(sizeof(PCD));
	pcd->data = NULL;
	pcd->float_data = NULL;
	pcd->fields = NULL;
	while (fgets(buf, 256, f)) {
		switch (*fm) {
		case VERSION:
			if (strncmp(buf, "VERSION", 7) != 0) break;
			fm++;
			break;
		case FIELDS:
			if (strncmp(buf, "FIELDS", 6) != 0) break;
			fm++;
			break;
		case SIZE:
			if (strncmp(buf, "SIZE", 4) != 0) break;
			fm++;
			break;
		case TYPE:
			if (strncmp(buf, "TYPE", 4) != 0) break;
			fm++;
			break;
		case COUNT:
			if (strncmp(buf, "COUNT", 5) != 0) break;
			fm++;
			break;
		case WIDTH:
			if (sscanf(buf, "WIDTH %d", &pcd->width) != 1) break;
			fm++;
			break;
		case HEIGHT:
			if (sscanf(buf, "HEIGHT %d", &pcd->height) != 1) break;
			fm++;
			break;
		case VIEWPOINT:
			if (sscanf(buf, "VIEWPOINT %f %f %f %f %f %f %f", &pcd->viewpoint[0], &pcd->viewpoint[1], &pcd->viewpoint[2],
				&pcd->viewpoint[3], &pcd->viewpoint[4], &pcd->viewpoint[5], &pcd->viewpoint[6]) != 7) break;
			pcd->camera.position.r = 0;
			pcd->camera.focal_length = DEFAULT_FOCAL_LENGTH;
			pcd->camera.position.i = (float)pcd->viewpoint[0];
			pcd->camera.position.j = (float)pcd->viewpoint[1];
			pcd->camera.position.k = (float)pcd->viewpoint[2];
			pcd->camera.rotation.r = (float)pcd->viewpoint[3];
			pcd->camera.rotation.i = (float)pcd->viewpoint[4];
			pcd->camera.rotation.j = (float)pcd->viewpoint[5];
			pcd->camera.rotation.k = (float)pcd->viewpoint[6];
			fm++;
			break;
		case POINTS:
			if (sscanf(buf, "POINTS %d", &pcd->numPoints) != 1) break;
			fm++;
			break;
		case DATA:
			if (strcmp(buf, "DATA ascii") == 0) pcd->data_storage = ASCII;
			else if (strcmp(buf, "DATA binary") == 0) pcd->data_storage = BINARY;
			else break;
			fm++;
			break;
		case POINT_CLOUD:
			break;
		default:
			break;
		}
	}
	fclose(f);
	return pcd;
}

PCD* NewPCD4FromFile(char* fileName) {
	FILE* f = fopen(fileName, "r");
	if (!f) {
		printf("File not found: %s\n", fileName);
		return NULL;
	}
	char buf[256];
	PCD* pcd = (PCD*)malloc(sizeof(PCD));
	pcd->numFields = 4;
	pcd->data = NULL;
	pcd->float_data = NULL;
	pcd->fields = NULL;
	int pointsParsed = 0;
	while (fgets(buf, 256, f)) {
		if (sscanf(buf, "POINTS %d", &pcd->numPoints) == 1) {
			pcd->float_data = (float*)malloc(4 * pcd->numPoints * sizeof(float));
		}
		else if (sscanf(buf, "VIEWPOINT %f %f %f %f %f %f %f", &pcd->viewpoint[0], &pcd->viewpoint[1], &pcd->viewpoint[2],
			&pcd->viewpoint[3], &pcd->viewpoint[4], &pcd->viewpoint[5], &pcd->viewpoint[6]) == 7) {
			pcd->camera.position.r = 0;
			pcd->camera.focal_length = DEFAULT_FOCAL_LENGTH;
			pcd->camera.position.i = (float)pcd->viewpoint[0];
			pcd->camera.position.j = (float)pcd->viewpoint[1];
			pcd->camera.position.k = (float)pcd->viewpoint[2];
			pcd->camera.rotation.r = (float)pcd->viewpoint[3];
			pcd->camera.rotation.i = (float)pcd->viewpoint[4];
			pcd->camera.rotation.j = (float)pcd->viewpoint[5];
			pcd->camera.rotation.k = (float)pcd->viewpoint[6];
		}
		else if (sscanf(buf, "%f %f %f %f", &pcd->float_data[pointsParsed * 4], &pcd->float_data[pointsParsed * 4 + 1],
			&pcd->float_data[pointsParsed * 4 + 2], &pcd->float_data[pointsParsed * 4 + 3]) == 4) {
			pointsParsed++;
		}
	}
	if (verboseLogging)
		printf("Parsed %s (%d points)\n", fileName, pointsParsed);
	fclose(f);
	return pcd;
}

void DeletePCD(PCD* pcd) {
	if (pcd) {
		if (pcd->fields) free(pcd->fields);
		if (pcd->data) free(pcd->data);
		if (pcd->float_data) free(pcd->float_data);
		free(pcd);
	}
}


Bitmap* NewBitmap(unsigned int width, unsigned int height, unsigned short bitsPerPixel) {
	Bitmap* bmp = (Bitmap*)malloc(sizeof(Bitmap));
	bmp->width = width;
	bmp->height = height;
	bmp->bitsPerPixel = bitsPerPixel;
	bmp->imageSize = bmp->width * bmp->height * bmp->bitsPerPixel / 8;
	bmp->data = (char*)malloc(bmp->imageSize);
	return bmp;
}

void DeleteBitmap(Bitmap* bmp) {
	if (bmp) {
		if (bmp->data) free(bmp->data);
		free(bmp);
	}
}

void SaveBitmap(Bitmap* bmp, char* fileName) {
	if (!bmp) return;
	FILE* f = fopen(fileName, "wb");
	if (!f) {
		printf("File not found: %s\n", fileName);
	}
	else {
		int i;
		fwrite("BM", 1, 2, f);
		int2byte(54 + bmp->imageSize, 4, f);
		fwrite("\0\0\0\0", 1, 4, f);
		int2byte(54, 4, f);
		int2byte(40, 4, f);
		int2byte(bmp->width, 4, f);
		int2byte(bmp->height, 4, f);
		int2byte(1, 2, f);
		int2byte(bmp->bitsPerPixel, 2, f);
		fwrite("\0\0\0\0", 1, 4, f);
		int2byte(bmp->imageSize, 4, f);
		for (i = 0; i < 16; i++) fputc('\0', f);
		fwrite(bmp->data, 1, bmp->imageSize, f);
		fclose(f);
	}
}

int SetRGB(Bitmap* bmp, int x, int y, int r, int g, int b) {
	if (!bmp || bmp->bitsPerPixel != 24 || x < 0 || y < 0 || x >= bmp->width || y >= bmp->height) return 0;
	int offset = (x + (bmp->height - 1 - y)*bmp->width) * 3;
	bmp->data[offset] = (unsigned char)(b);
	bmp->data[offset + 1] = (unsigned char)(g);
	bmp->data[offset + 2] = (unsigned char)(r);
	return 1;
}

int ShowXYProjection(Bitmap* bmp, PCD* pcd, int pointSize) {
	int i, j, k;
	int rgb, r, g, b;
	int pixelsDrawn = 0;
	float minX = pcd->float_data[0];
	float maxX = pcd->float_data[0];
	float minY = pcd->float_data[1];
	float maxY = pcd->float_data[1];
	float x, y, scaleX, scaleY, offsetX, offsetY;
	for (i = 1; i < pcd->numPoints; i++) {
		if (pcd->float_data[i * 4 + 3] != 0){
			if (pcd->float_data[i * 4] < minX) minX = pcd->float_data[i * 4];
			if (pcd->float_data[i * 4] > maxX) maxX = pcd->float_data[i * 4];
			if (pcd->float_data[i * 4 + 1] < minY) minY = pcd->float_data[i * 4 + 1];
			if (pcd->float_data[i * 4 + 1] > maxY) maxY = pcd->float_data[i * 4 + 1];
		}
	}
	if ((maxX - minX) >(maxY - minY)) {
		scaleX = (bmp->width - 1) / (maxX - minX) * (1 - MARGIN * 2);
		scaleY = (bmp->height - 1) / (maxX - minX)* (1 - MARGIN * 2);
		offsetX = (bmp->width - 1)*MARGIN;
		offsetY = (bmp->height - 1)*MARGIN + (bmp->height - 1 - (maxY - minY)*scaleY) / 2;
	}
	else {
		scaleX = (bmp->width - 1) / (maxY - minY)* (1 - MARGIN * 2);
		scaleY = (bmp->height - 1) / (maxY - minY)* (1 - MARGIN * 2);
		offsetX = (bmp->width - 1)*MARGIN + (bmp->width - 1 - (maxX - minX)*scaleX) / 2;
		offsetY = (bmp->height - 1)*MARGIN;
	}
	for (i = 0; i < pcd->numPoints; i++) {
		if (pcd->float_data[i * 4 + 3] != 0){
			rgb = (int)pcd->float_data[i * 4 + 3];
			r = (rgb >> 16) & 0xFF;
			g = (rgb >> 8) & 0xFF;
			b = rgb & 0xFF;
			x = (pcd->float_data[i * 4] - minX) * scaleX + offsetX;
			y = (pcd->float_data[i * 4 + 1] - minY) * scaleY + offsetY;
			x -= (pointSize - 1) / 2;
			y -= (pointSize - 1) / 2;
			//printf("%f %f\n", x, y);
			for (j = 0; j < pointSize; j++) {
				for (k = 0; k < pointSize; k++) {
					pixelsDrawn += SetRGB(bmp, (int)x + j, (int)y + k, r, g, b);
				}
			}

		}
	}
	return pixelsDrawn;
}

int Show3DProjection(Bitmap* bmp, PCD* pcd, int pointSize, int zoomToFit) {
	int i, j, k;
	Quaternion q, newPoint;
	q.r = 0;
	if (verboseLogging)
		printf("Using camera: (%f %f %f %f %f %f %f)\n", pcd->camera.position.i, pcd->camera.position.j, pcd->camera.position.k,
			pcd->camera.rotation.r, pcd->camera.rotation.i, pcd->camera.rotation.j, pcd->camera.rotation.k);
	for (i = 0; i < pcd->numPoints; i++) {
		//first transform point cloud to camera coordinates
		q.i = pcd->float_data[i * 4] - pcd->camera.position.i;
		q.j = pcd->float_data[i * 4 + 1] - pcd->camera.position.j;
		q.k = pcd->float_data[i * 4 + 2] - pcd->camera.position.k;
		newPoint = quaternionRot(pcd->camera.rotation, q);
		//next map points to image plane based on camera focal length
		if (newPoint.k > pcd->camera.focal_length) { //if not behind camera
			pcd->float_data[i * 4] = newPoint.i * pcd->camera.focal_length / newPoint.k;
			pcd->float_data[i * 4 + 1] = newPoint.j * pcd->camera.focal_length / newPoint.k;
		}
		else {
			pcd->float_data[i * 4 + 3] = 0; //erase point
		}
	}
	//lastly plot these points on xy plane of bitmap
	int rgb, r, g, b;
	int pixelsDrawn = 0;
	float maxX = -1;
	float maxY = -1;
	float x, y, scaleX, scaleY;
	if (zoomToFit){
		for (i = 0; i < pcd->numPoints; i++) {
			if (pcd->float_data[i * 4 + 3] != 0){
				if (fabs(pcd->float_data[i * 4]) > maxX) {
					maxX = fabs(pcd->float_data[i * 4]);
				}
				if (fabs(pcd->float_data[i * 4 + 1]) > maxY) {
					maxY = fabs(pcd->float_data[i * 4 + 1]);
				}
			}
		}
	}
	else {
		maxX = 1;
		maxY = 1;
	}
	if (verboseLogging)
		printf("display rectangle: %f x %f\n", maxX, maxY);
	if (maxX > maxY) {
		scaleX = (bmp->width - 1) * (1 - MARGIN * 2) / (maxX * 2);
		scaleY = (bmp->height - 1) * (1 - MARGIN * 2) / (maxX * 2);
	}
	else {
		scaleX = (bmp->width - 1) * (1 - MARGIN * 2) / (maxY * 2);
		scaleY = (bmp->height - 1) * (1 - MARGIN * 2) / (maxY * 2);
	}
	for (i = 0; i < pcd->numPoints; i++) {
		if (pcd->float_data[i * 4 + 3] != 0){
			//printf("%f %f\n",pcd->float_data[i * 4],pcd->float_data[i * 4 + 1]);
			rgb = (int)pcd->float_data[i * 4 + 3];
			r = (rgb >> 16) & 0xFF;
			g = (rgb >> 8) & 0xFF;
			b = rgb & 0xFF;
			x = pcd->float_data[i * 4] * scaleX + bmp->width / 2;
			y = pcd->float_data[i * 4 + 1] * scaleY + bmp->height / 2;
			x -= (pointSize - 1) / 2;
			y -= (pointSize - 1) / 2;
			//printf("%f %f\n", x, y);
			for (j = 0; j < pointSize; j++) {
				for (k = 0; k < pointSize; k++) {
					pixelsDrawn += SetRGB(bmp, (int)x + j, (int)y + k, r, g, b);
				}
			}
		}
	}
	return pixelsDrawn;

}

void pcd2bmp(char* src, char* dest, int pointSize) {
	PCD* pcd = NewPCD4FromFile(src);
	if (!pcd) return;
	Bitmap* bmp = NewBitmap(500, 500, 24);
	if (!bmp) return;
	memset(bmp->data, 0, bmp->imageSize);
	int pixelsDrawn = Show3DProjection(bmp, pcd, pointSize, 0);
	if (pixelsDrawn <= 0) {
		pixelsDrawn = Show3DProjection(bmp, pcd, pointSize, 1);
	}
	if (pixelsDrawn > 0)
		SaveBitmap(bmp, dest);
	DeleteBitmap(bmp);
	DeletePCD(pcd);
}

void printUsage() {
	printf("3DProjection: Projects point cloud data to a bitmap\n");
	printf("Usage: 3DProjection [-c x,y,z] [-r q1,q2,q3,q4] [-f fl] [-p psz] [-z] [-v] <src.pcd> <dest.bmp>\n");
	printf("	-c (camera coordinates)\n");
	printf("	-r (camera rotation quaternion)\n");
	printf("	-f (camera focal length)\n");
	printf("	-p (point size)\n");
	printf("	-z (zoom out to fit points)\n");
	printf("	-v (verbose logging)\n");
}

int main(int argc, char* argv[])
{
	int i;
	char *src = NULL, *dest = NULL;
	float camera_x, camera_y, camera_z, camera_q1, camera_q2, camera_q3, camera_q4, focal_length;
	int isSetXYZ = 0, isSetRotation = 0, isSetFocalLength = 0, zoomToFit = 0;
	int pointSize = 1;
	for (i = 1; i < argc; i++) {
		if (i != argc - 1){
			if (strcmp(argv[i], "-c") == 0) {
				if (sscanf(argv[i + 1], "%f,%f,%f", &camera_x, &camera_y, &camera_z) == 3) {
					isSetXYZ = 1;
					i++;
				}
				else {
					printf("Invalid argument for option -c\n");
					printUsage();
					return 1;
				}
			}
			else if (strcmp(argv[i], "-r") == 0) {
				if (sscanf(argv[i + 1], "%f,%f,%f,%f", &camera_q1, &camera_q2, &camera_q3, &camera_q4) == 4) {
					isSetRotation = 1;
					i++;
				}
				else {
					printf("Invalid argument for option -r\n");
					printUsage();
					return 1;
				}
			}
			else if (strcmp(argv[i], "-f") == 0) {
				if (sscanf(argv[i + 1], "%f", &focal_length) == 1) {
					isSetFocalLength = 1;
					i++;
				}
				else {
					printf("Invalid argument for option -f\n");
					printUsage();
					return 1;
				}
			}
			else if (strcmp(argv[i], "-p") == 0) {
				if (sscanf(argv[i + 1], "%d", &pointSize) == 1) {
					i++;
				}
				else {
					printf("Invalid argument for option -p\n");
					printUsage();
					return 1;
				}
			}
			else if (strcmp(argv[i], "-v") == 0) {
				verboseLogging = 1;
			}
			else if (strcmp(argv[i], "-z") == 0) {
				zoomToFit = 1;
			}
			else if (!src) {
				src = argv[i];
			}
			else if (!dest) {
				dest = argv[i];
			}
		}
		else if (strcmp(argv[i], "-v") == 0) {
			verboseLogging = 1;
		}
		else if (strcmp(argv[i], "-z") == 0) {
			zoomToFit = 1;
		}
		else if (!src) {
			src = argv[i];
		}
		else if (!dest) {
			dest = argv[i];
		}
	}

	if (!src || !dest) {
		printUsage();
		return 1;
	}

	PCD* pcd = NewPCD4FromFile(src);
	if (!pcd) return 1;

	if (isSetXYZ) {
		pcd->camera.position.i = camera_x;
		pcd->camera.position.j = camera_y;
		pcd->camera.position.k = camera_z;
	}
	if (isSetRotation) {
		pcd->camera.rotation.r = camera_q1;
		pcd->camera.rotation.i = camera_q2;
		pcd->camera.rotation.j = camera_q3;
		pcd->camera.rotation.k = camera_q4;
	}
	if (isSetFocalLength) {
		pcd->camera.focal_length = focal_length;
	}

	Bitmap* bmp = NewBitmap(500, 500, 24);
	if (!bmp) return 1;
	memset(bmp->data, 0, bmp->imageSize); //clear bitmap to all black
	//ShowXYProjection(bmp, pcd, 5);
	int pixelsDrawn = Show3DProjection(bmp, pcd, pointSize, zoomToFit);
	if (pixelsDrawn <= 0 && !zoomToFit) { //try again with zoom
		pixelsDrawn = Show3DProjection(bmp, pcd, pointSize, 1);
	}
	if (verboseLogging)
		printf("%d pixels drawn\n", pixelsDrawn);
	if (pixelsDrawn > 0)
		SaveBitmap(bmp, dest);

	DeleteBitmap(bmp);
	DeletePCD(pcd);
	return 0;
}
